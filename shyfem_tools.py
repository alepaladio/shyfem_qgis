from qgis.PyQt.QtCore import QSettings, QTranslator, QCoreApplication
from qgis.PyQt.QtGui import QIcon
from qgis.PyQt.QtWidgets import QAction, QFileDialog, QMessageBox
from qgis.core import (
    QgsProject, QgsGeometry, QgsPointXY, QgsWkbTypes, 
    QgsVectorLayer, QgsFeature, QgsField, QgsFields,
    QgsPoint, QgsCoordinateReferenceSystem, QgsLayerTreeGroup
)
from qgis.PyQt.QtCore import QVariant
import os

class ShyfemTools:
    def __init__(self, iface):
        self.iface = iface
        self.actions = []
        
    def initGui(self):
        # Get the plugin directory
        plugin_dir = os.path.dirname(__file__)
        
        # Create Export action
        export_icon = QIcon(os.path.join(plugin_dir, 'export.png'))
        self.export_action = QAction(export_icon, "Export to SHYFEM", self.iface.mainWindow())
        self.export_action.triggered.connect(self.export_run)
        self.export_action.setStatusTip("Export current layer to SHYFEM .grd format")
        
        # Create Import action  
        import_icon = QIcon(os.path.join(plugin_dir, 'import.png'))
        self.import_action = QAction(import_icon, "Import from SHYFEM", self.iface.mainWindow())
        self.import_action.triggered.connect(self.import_run)
        self.import_action.setStatusTip("Import SHYFEM .grd file to QGIS")
        
        # Create Save Modified Grid action
        save_icon = QIcon(os.path.join(plugin_dir, 'save_new_grd.svg'))
        self.save_action = QAction(save_icon, "Save Modified Grid", self.iface.mainWindow())
        self.save_action.triggered.connect(self.save_modified_grid)
        self.save_action.setStatusTip("Save modified points with original element connectivity")
        
        # Add to Vector Menu (under the main Plugins menu)
        self.iface.addPluginToVectorMenu("SHYFEM Tools", self.export_action)
        self.iface.addPluginToVectorMenu("SHYFEM Tools", self.import_action)
        self.iface.addPluginToVectorMenu("SHYFEM Tools", self.save_action)
        
        # Add to Toolbar
        self.iface.addToolBarIcon(self.export_action)
        self.iface.addToolBarIcon(self.import_action)
        self.iface.addToolBarIcon(self.save_action)
        
        # Store actions for cleanup
        self.actions = [self.export_action, self.import_action, self.save_action]
        
    def unload(self):
        # Remove from menu and toolbar
        self.iface.removePluginVectorMenu("SHYFEM Tools", self.export_action)
        self.iface.removePluginVectorMenu("SHYFEM Tools", self.import_action)
        self.iface.removePluginVectorMenu("SHYFEM Tools", self.save_action)
        
        for action in self.actions:
            self.iface.removeToolBarIcon(action)
    
    # ==================== EXPORT FUNCTIONS ====================
    def export_run(self):
        """Export function - called when export button is clicked"""
        layer = self.iface.activeLayer()
        
        if not layer or layer.geometryType() not in [QgsWkbTypes.LineGeometry, QgsWkbTypes.PolygonGeometry]:
            QMessageBox.warning(self.iface.mainWindow(), "Error", 
                              "Please select a line or polygon vector layer!")
            return
        
        # Generate automatic filename based on layer name
        layer_name = layer.name().replace(' ', '_').lower()
        project_path = QgsProject.instance().homePath()
        
        if not project_path:
            project_path = os.path.expanduser("~")
            
        auto_filename = os.path.join(project_path, f"{layer_name}_000.grd")
        
        # Get output file with suggested name
        filename, _ = QFileDialog.getSaveFileName(
            self.iface.mainWindow(), 
            "Export to SHYFEM .grd", 
            auto_filename, 
            "GRD Files (*.grd)"
        )
        
        if not filename:
            return
            
        # Export the data
        success = self.export_to_shyfem(layer, filename)
        if success:
            QMessageBox.information(self.iface.mainWindow(), "Success", 
                                  f"Exported {layer.featureCount()} features to:\n{filename}")
        else:
            QMessageBox.warning(self.iface.mainWindow(), "Error", 
                              "Failed to export data")
    
    def get_coordinate_precision(self, layer):
        """Determine coordinate precision based on CRS"""
        crs = layer.crs()
        if crs.isGeographic():
            return 8  # 8 decimals for lat/lon
        else:
            return 2  # 2 decimals for projected coordinates
    
    def extract_vertices_from_geometry(self, geometry):
        """Extract all vertices from geometry, handling both lines and polygons"""
        vertices = []
        
        if geometry.isMultipart():
            # Handle multi-part geometries
            if geometry.type() == QgsWkbTypes.LineGeometry:
                multi_line = geometry.asMultiPolyline()
                for line in multi_line:
                    vertices.extend(line)
            elif geometry.type() == QgsWkbTypes.PolygonGeometry:
                multi_polygon = geometry.asMultiPolygon()
                for polygon in multi_polygon:
                    for ring in polygon:
                        vertices.extend(ring)
        else:
            # Handle single-part geometries
            if geometry.type() == QgsWkbTypes.LineGeometry:
                vertices = geometry.asPolyline()
            elif geometry.type() == QgsWkbTypes.PolygonGeometry:
                polygon = geometry.asPolygon()
                for ring in polygon:
                    vertices.extend(ring)
        
        return vertices
    
    def split_point_ids_for_line_wrapping(self, point_ids, max_columns=132):
        """Split point IDs into multiple lines if they exceed max_columns limit"""
        lines = []
        current_line = []
        current_length = 0
        
        for point_id in point_ids:
            point_str = str(point_id)
            # +1 for the space that will be added before this point
            point_length = len(point_str) + (1 if current_line else 0)
            
            # Check if adding this point would exceed the column limit
            if current_length + point_length > max_columns and current_line:
                lines.append(current_line)
                current_line = [point_str]
                current_length = len(point_str)
            else:
                if current_line:
                    current_length += point_length
                else:
                    current_length = len(point_str)
                current_line.append(point_str)
        
        if current_line:
            lines.append(current_line)
        
        return lines
    
    def export_to_shyfem(self, layer, filename):
        """Export line or polygon features to SHYFEM format"""
        try:
            all_points = []
            line_connections = []
            point_id_map = {}  # Maps (x,y) to point ID
            
            point_counter = 1
            line_counter = 1
            
            # Get coordinate precision based on CRS
            precision = self.get_coordinate_precision(layer)
            
            # Process each feature in the layer
            for feature in layer.getFeatures():
                geometry = feature.geometry()
                
                # Extract all vertices from the geometry
                vertices = self.extract_vertices_from_geometry(geometry)
                
                if not vertices:
                    continue
                    
                line_point_ids = []
                
                # Process each vertex
                for vertex in vertices:
                    point = (vertex.x(), vertex.y())
                    
                    # Check if point already exists
                    if point not in point_id_map:
                        point_id_map[point] = point_counter
                        all_points.append((point_counter, point[0], point[1]))
                        point_counter += 1
                    
                    line_point_ids.append(point_id_map[point])
                
                # For polygons, ensure the ring is closed (first and last points connected)
                if layer.geometryType() == QgsWkbTypes.PolygonGeometry:
                    if line_point_ids and line_point_ids[0] != line_point_ids[-1]:
                        line_point_ids.append(line_point_ids[0])
                
                # Add line/polygon connection
                if line_point_ids:  # Only add if we have points
                    line_connections.append((line_counter, line_point_ids))
                    line_counter += 1
            
            # Write to file
            with open(filename, 'w') as f:
                # Write points section
                for point_id, x, y in all_points:
                    f.write(f"1 {point_id} 0 {x:.{precision}f} {y:.{precision}f}\n")
                
                if line_connections:  # Only add newline if we have connections
                    f.write("\n")
                
                # Write line connections with column wrapping
                for line_id, point_ids in line_connections:
                    # Split point IDs into multiple lines if they exceed column limit
                    point_lines = self.split_point_ids_for_line_wrapping(point_ids)
                    
                    # Write the line header with total number of points
                    f.write(f"3 {line_id} 0 {len(point_ids)}\n")
                    
                    # Write each line of point IDs
                    for i, point_line in enumerate(point_lines):
                        line_content = " " + " ".join(point_line)
                        f.write(line_content)
                        # Only add newline if this is not the last line
                        if i < len(point_lines) - 1:
                            f.write("\n")
                    f.write("\n")
            
            return True
            
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Export Error", f"Error during export: {str(e)}")
            return False

    # ==================== IMPORT FUNCTIONS ====================
    def import_run(self):
        """Import function - called when import button is clicked"""
        # Get input file
        filename, _ = QFileDialog.getOpenFileName(
            self.iface.mainWindow(), 
            "Import SHYFEM .grd file", 
            "", 
            "GRD Files (*.grd)"
        )
        
        if not filename:
            return
            
        success = self.import_shyfem_to_qgis(filename)
        if success:
            QMessageBox.information(self.iface.mainWindow(), "Success", 
                                  f"Imported SHYFEM data from:\n{filename}")
        else:
            QMessageBox.warning(self.iface.mainWindow(), "Error", 
                              "Failed to import SHYFEM file")
    
    def parse_shyfem_file(self, filename):
        """Parse SHYFEM .grd file and extract points, lines, and polygons"""
        points = {}      # {point_id: (x, y, point_type, z)}
        lines = []       # [(line_id, line_type, point_ids)]
        polygons = []    # [(polygon_id, element_type, num_points, node_ids, depth)]
        
        try:
            with open(filename, 'r') as f:
                content = f.read()
            
            lines_content = content.strip().split('\n')
            i = 0
            
            while i < len(lines_content):
                line = lines_content[i].strip()
                if not line:
                    i += 1
                    continue
                    
                parts = line.split()
                
                # Parse points (starting with '1')
                if len(parts) >= 5 and parts[0] == '1':
                    # Point line: 1 point_id point_type x y [z]
                    point_id = int(parts[1])
                    point_type = int(parts[2])
                    x = float(parts[3])
                    y = float(parts[4])
                    # Z value is optional - check if it exists
                    z = None
                    if len(parts) >= 6:
                        try:
                            z = float(parts[5])
                        except ValueError:
                            z = None
                    
                    points[point_id] = (x, y, point_type, z)
                    i += 1
                
                # Parse polygons (starting with '2')  
                elif len(parts) >= 7 and parts[0] == '2':
                    # Polygon line: 2 polygon_id element_type num_points node1 node2 node3 [depth]
                    polygon_id = int(parts[1])
                    element_type = int(parts[2])
                    num_points = int(parts[3])
                    
                    # Extract node IDs (the next 'num_points' values)
                    node_ids = []
                    for j in range(4, 4 + num_points):
                        if j < len(parts):
                            node_ids.append(int(parts[j]))
                    
                    # Depth is optional - check if it exists
                    depth_index = 4 + num_points
                    depth = 0.0  # Default value if depth is missing
                    if depth_index < len(parts):
                        try:
                            depth = float(parts[depth_index])
                        except ValueError:
                            depth = 0.0
                    
                    polygons.append((polygon_id, element_type, num_points, node_ids, depth))
                    i += 1
                
                # Parse lines (starting with '3')
                elif len(parts) >= 4 and parts[0] == '3':
                    # Line definition: 3 line_id line_type num_points
                    line_id = int(parts[1])
                    line_type = int(parts[2])
                    num_points = int(parts[3])
                    
                    # Collect point IDs from subsequent lines until we have all points
                    collected_points = []
                    current_line = i + 1
                    
                    while len(collected_points) < num_points and current_line < len(lines_content):
                        point_line = lines_content[current_line].strip()
                        if point_line:
                            line_points = list(map(int, point_line.split()))
                            collected_points.extend(line_points)
                        current_line += 1
                    
                    if len(collected_points) == num_points:
                        lines.append((line_id, line_type, collected_points))
                    else:
                        print(f"Warning: Expected {num_points} points, got {len(collected_points)} for line {line_id}")
                    
                    i = current_line
                else:
                    i += 1
                    
        except Exception as e:
            print(f"Error parsing SHYFEM file: {e}")
            return None, None, None
            
        return points, lines, polygons

    def create_point_layer(self, points, base_name):
        """Create a point layer from the parsed points"""
        # Create point layer
        point_layer = QgsVectorLayer("Point", f"{base_name}_points", "memory")
        provider = point_layer.dataProvider()
        
        # Add fields
        fields = QgsFields()
        fields.append(QgsField("point_id", QVariant.Int))
        fields.append(QgsField("point_type", QVariant.Int))
        fields.append(QgsField("x", QVariant.Double))
        fields.append(QgsField("y", QVariant.Double))
        fields.append(QgsField("z", QVariant.Double))  # Add Z field
        provider.addAttributes(fields)
        point_layer.updateFields()
        
        # Add features
        features = []
        for point_id, (x, y, point_type, z) in points.items():
            feature = QgsFeature()
            feature.setGeometry(QgsGeometry.fromPointXY(QgsPointXY(x, y)))
            # Use 0.0 as default if z is None
            z_value = z if z is not None else 0.0
            feature.setAttributes([point_id, point_type, x, y, z_value])
            features.append(feature)
        
        provider.addFeatures(features)
        point_layer.updateExtents()
        
        return point_layer
        
    def create_line_layer(self, lines, points_dict, base_name):
        """Create a line layer from the parsed lines"""
        # Create line layer
        line_layer = QgsVectorLayer("LineString", f"{base_name}_lines", "memory")
        provider = line_layer.dataProvider()
        
        # Add fields
        fields = QgsFields()
        fields.append(QgsField("line_id", QVariant.Int))
        fields.append(QgsField("line_type", QVariant.Int))
        fields.append(QgsField("num_points", QVariant.Int))
        provider.addAttributes(fields)
        line_layer.updateFields()
        
        # Add features
        features = []
        for line_id, line_type, point_ids in lines:
            # Create line geometry from point IDs
            line_points = []
            for pid in point_ids:
                if pid in points_dict:
                    x, y, _, _ = points_dict[pid]  # Fixed: now expects 4 values
                    line_points.append(QgsPoint(x, y))
            
            if len(line_points) >= 2:
                line_geometry = QgsGeometry.fromPolyline(line_points)
                feature = QgsFeature()
                feature.setGeometry(line_geometry)
                feature.setAttributes([line_id, line_type, len(point_ids)])
                features.append(feature)
        
        provider.addFeatures(features)
        line_layer.updateExtents()
        
        return line_layer
    
    def create_polygon_layer(self, polygons, points_dict, base_name):
        """Create a polygon layer from the parsed polygons"""
        # Create polygon layer
        polygon_layer = QgsVectorLayer("Polygon", f"{base_name}_polygons", "memory")
        provider = polygon_layer.dataProvider()
        
        # Add fields - UPDATED: Added p1, p2, p3 fields for node connectivity
        fields = QgsFields()
        fields.append(QgsField("polygon_id", QVariant.Int))
        fields.append(QgsField("element_type", QVariant.Int))
        fields.append(QgsField("num_points", QVariant.Int))
        fields.append(QgsField("p1", QVariant.Int))  # First node ID
        fields.append(QgsField("p2", QVariant.Int))  # Second node ID
        fields.append(QgsField("p3", QVariant.Int))  # Third node ID
        fields.append(QgsField("depth", QVariant.Double))
        provider.addAttributes(fields)
        polygon_layer.updateFields()
        
        # Add features
        features = []
        polygons_created = 0
        polygons_skipped = 0
        
        for polygon_id, element_type, num_points, node_ids, depth in polygons:
            # Create polygon geometry from node IDs
            polygon_points = []
            valid_nodes = True
            
            # Get coordinates for each node
            for node_id in node_ids:
                if node_id in points_dict:
                    x, y, _, _ = points_dict[node_id]  # Fixed: now expects 4 values
                    polygon_points.append(QgsPointXY(x, y))
                else:
                    print(f"Warning: Node {node_id} not found in points for polygon {polygon_id}")
                    valid_nodes = False
                    break
            
            # Only create polygon if all nodes are valid and we have at least 3 points
            if valid_nodes and len(polygon_points) >= 3:
                # Close the polygon by adding the first point at the end
                if polygon_points[0] != polygon_points[-1]:
                    polygon_points.append(polygon_points[0])
                
                # Create polygon geometry
                polygon_geometry = QgsGeometry.fromPolygonXY([polygon_points])
                feature = QgsFeature()
                feature.setGeometry(polygon_geometry)
                
                # Store node IDs in p1, p2, p3 fields
                p1 = node_ids[0] if len(node_ids) > 0 else 0
                p2 = node_ids[1] if len(node_ids) > 1 else 0
                p3 = node_ids[2] if len(node_ids) > 2 else 0
                
                feature.setAttributes([polygon_id, element_type, num_points, p1, p2, p3, depth])
                features.append(feature)
                polygons_created += 1
            else:
                polygons_skipped += 1
        
        if features:
            provider.addFeatures(features)
            polygon_layer.updateExtents()
            print(f"DEBUG: Created {polygons_created} polygons, skipped {polygons_skipped} polygons")
        else:
            print(f"DEBUG: No valid polygons created, skipped {polygons_skipped} polygons")
        
        return polygon_layer
    
    def import_shyfem_to_qgis(self, filename):
        """Main import function"""
        try:
            # Parse the GRD file (now returns points, lines, AND polygons)
            points, lines, polygons = self.parse_shyfem_file(filename)
            
            if points is None or lines is None or polygons is None:
                QMessageBox.critical(self.iface.mainWindow(), "Parse Error", "Failed to parse the GRD file")
                return False
            
            print(f"DEBUG: Found {len(points)} points, {len(lines)} lines, {len(polygons)} polygons")
            
            # Get base name for layers
            base_name = os.path.splitext(os.path.basename(filename))[0]
            
            # Create group for the imported data
            root = QgsProject.instance().layerTreeRoot()
            group = root.insertGroup(0, f"SHYFEM_{base_name}")
            
            layers_added = []
            
            # Create point layer if we have points
            if points:
                point_layer = self.create_point_layer(points, base_name)
                QgsProject.instance().addMapLayer(point_layer, False)
                group.addLayer(point_layer)
                layers_added.append(point_layer)
                print("DEBUG: Point layer created successfully")
            
            # Create line layer if we have lines
            if lines:
                line_layer = self.create_line_layer(lines, points, base_name)
                QgsProject.instance().addMapLayer(line_layer, False)
                group.addLayer(line_layer)
                layers_added.append(line_layer)
                print("DEBUG: Line layer created successfully")
            
            # Create polygon layer if we have polygons
            if polygons:
                print(f"DEBUG: Creating polygon layer with {len(polygons)} polygons")
                polygon_layer = self.create_polygon_layer(polygons, points, base_name)
                
                # Only add polygon layer if it has features
                if polygon_layer.featureCount() > 0:
                    QgsProject.instance().addMapLayer(polygon_layer, False)
                    group.addLayer(polygon_layer)
                    layers_added.append(polygon_layer)
                    print("DEBUG: Polygon layer created successfully")
                else:
                    print("DEBUG: Polygon layer created but has no features - skipping")
            
            # Zoom to extent if we have layers
            if layers_added:
                # Combine extents of all layers
                full_extent = None
                for layer in layers_added:
                    if full_extent is None:
                        full_extent = layer.extent()
                    else:
                        full_extent.combineExtentWith(layer.extent())
                
                self.iface.mapCanvas().setExtent(full_extent)
                self.iface.mapCanvas().refresh()
            
            total_polygons = polygon_layer.featureCount() if polygons and 'polygon_layer' in locals() else 0
            print(f"Imported: {len(points)} points, {len(lines)} lines, {total_polygons} polygons")
            return True
            
        except Exception as e:
            import traceback
            error_details = traceback.format_exc()
            print(f"DEBUG - Full error traceback:\n{error_details}")
            QMessageBox.critical(self.iface.mainWindow(), "Import Error", 
                               f"Error during import: {str(e)}\n\nDetails:\n{error_details}")
            return False

    # ==================== SAVE MODIFIED GRID FUNCTION ====================
    def save_modified_grid(self):
        """Save modified grid with original connectivity"""
        # Find SHYFEM group and layers
        root = QgsProject.instance().layerTreeRoot()
        shyfem_groups = []
        
        # Find all SHYFEM groups
        for child in root.children():
            if isinstance(child, QgsLayerTreeGroup) and child.name().startswith('SHYFEM_'):
                shyfem_groups.append(child)
        
        if not shyfem_groups:
            QMessageBox.warning(self.iface.mainWindow(), "Error", 
                              "No SHYFEM groups found. Please import a GRD file first.")
            return
        
        # Let user select which SHYFEM group to save
        if len(shyfem_groups) == 1:
            selected_group = shyfem_groups[0]
        else:
            # TODO: Implement group selection dialog if multiple SHYFEM groups exist
            selected_group = shyfem_groups[0]
            QMessageBox.information(self.iface.mainWindow(), "Info", 
                                  f"Using group: {selected_group.name()}")
        
        # Find point and polygon layers in the selected group
        point_layer = None
        polygon_layer = None
        line_layer = None
        
        for layer_tree in selected_group.findLayers():
            layer = layer_tree.layer()
            if layer.name().endswith('_points'):
                point_layer = layer
            elif layer.name().endswith('_polygons'):
                polygon_layer = layer
            elif layer.name().endswith('_lines'):
                line_layer = layer
        
        if not point_layer or not polygon_layer:
            QMessageBox.warning(self.iface.mainWindow(), "Error", 
                              "Could not find both point and polygon layers in the SHYFEM group.")
            return
        
        # Get output filename
        base_name = selected_group.name().replace('SHYFEM_', '')
        project_path = QgsProject.instance().homePath()
        if not project_path:
            project_path = os.path.expanduser("~")
        
        suggested_filename = os.path.join(project_path, f"{base_name}_modified.grd")
        
        filename, _ = QFileDialog.getSaveFileName(
            self.iface.mainWindow(), 
            "Save Modified Grid", 
            suggested_filename, 
            "GRD Files (*.grd)"
        )
        
        if not filename:
            return
        
        # Save the modified grid
        success = self.save_modified_grid_to_file(point_layer, polygon_layer, line_layer, filename)
        if success:
            QMessageBox.information(self.iface.mainWindow(), "Success", 
                                  f"Saved modified grid to:\n{filename}")
        else:
            QMessageBox.warning(self.iface.mainWindow(), "Error", 
                              "Failed to save modified grid")
    
    def save_modified_grid_to_file(self, point_layer, polygon_layer, line_layer, filename):
        """Save modified grid with original connectivity to GRD file"""
        try:
            # Collect modified points with their IDs and coordinates
            points_data = {}
            for feature in point_layer.getFeatures():
                point_id = feature['point_id']
                geometry = feature.geometry()
                point = geometry.asPoint()
                point_type = feature['point_type']
                z_value = feature['z'] if 'z' in point_layer.fields().names() else 0.0
                points_data[point_id] = (point.x(), point.y(), point_type, z_value)
            
            # Collect polygons with their connectivity
            polygons_data = []
            for feature in polygon_layer.getFeatures():
                polygon_id = feature['polygon_id']
                element_type = feature['element_type']
                num_points = feature['num_points']
                # Use stored node IDs from p1, p2, p3 fields
                p1 = feature['p1']
                p2 = feature['p2'] 
                p3 = feature['p3']
                depth = feature['depth']
                
                polygons_data.append((polygon_id, element_type, num_points, [p1, p2, p3], depth))
            
            # Collect lines if they exist
            lines_data = []
            if line_layer:
                for feature in line_layer.getFeatures():
                    line_id = feature['line_id']
                    line_type = feature['line_type']
                    # For lines, we need to reconstruct the point IDs from geometry
                    # since we don't store the original connectivity for lines
                    geometry = feature.geometry()
                    if geometry.isMultipart():
                        vertices = geometry.asMultiPolyline()[0]
                    else:
                        vertices = geometry.asPolyline()
                    
                    # Find closest point IDs for each vertex
                    point_ids = []
                    for vertex in vertices:
                        closest_point_id = None
                        min_distance = float('inf')
                        for pid, (px, py, _, _) in points_data.items():
                            distance = ((vertex.x() - px) ** 2 + (vertex.y() - py) ** 2) ** 0.5
                            if distance < min_distance:
                                min_distance = distance
                                closest_point_id = pid
                        if closest_point_id and min_distance < 0.001:  # Tolerance for point matching
                            point_ids.append(closest_point_id)
                    
                    if len(point_ids) >= 2:
                        lines_data.append((line_id, line_type, point_ids))
            
            # Get coordinate precision from point layer CRS
            precision = self.get_coordinate_precision(point_layer)
            
            # Write to file
            with open(filename, 'w') as f:
                # Write points section
                for point_id, (x, y, point_type, z) in points_data.items():
                    if z != 0.0:  # Only write Z if it has a value
                        f.write(f"1 {point_id} {point_type} {x:.{precision}f} {y:.{precision}f} {z:.{precision}f}\n")
                    else:
                        f.write(f"1 {point_id} {point_type} {x:.{precision}f} {y:.{precision}f}\n")
                
                # Write polygons section if we have polygons
                if polygons_data:
                    f.write("\n")
                    for polygon_id, element_type, num_points, node_ids, depth in polygons_data:
                        f.write(f"2 {polygon_id} {element_type} {num_points}")
                        for node_id in node_ids:
                            f.write(f" {node_id}")
                        f.write(f" {depth:.6f}\n")
                
                # Write lines section if we have lines
                if lines_data:
                    f.write("\n")
                    for line_id, line_type, point_ids in lines_data:
                        # Split point IDs into multiple lines if they exceed column limit
                        point_lines = self.split_point_ids_for_line_wrapping(point_ids)
                        
                        # Write the line header with total number of points
                        f.write(f"3 {line_id} {line_type} {len(point_ids)}\n")
                        
                        # Write each line of point IDs
                        for i, point_line in enumerate(point_lines):
                            line_content = " " + " ".join(point_line)
                            f.write(line_content)
                            # Only add newline if this is not the last line
                            if i < len(point_lines) - 1:
                                f.write("\n")
                        f.write("\n")
            
            print(f"Saved modified grid: {len(points_data)} points, {len(polygons_data)} polygons, {len(lines_data)} lines")
            return True
            
        except Exception as e:
            import traceback
            error_details = traceback.format_exc()
            print(f"Error saving modified grid: {error_details}")
            QMessageBox.critical(self.iface.mainWindow(), "Save Error", 
                               f"Error saving modified grid: {str(e)}")
            return False