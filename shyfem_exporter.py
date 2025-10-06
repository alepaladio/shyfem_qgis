from qgis.PyQt.QtCore import QSettings, QTranslator, QCoreApplication
from qgis.PyQt.QtGui import QIcon
from qgis.PyQt.QtWidgets import QAction, QFileDialog, QMessageBox
from qgis.core import (
    QgsProject, QgsGeometry, QgsPointXY, QgsWkbTypes, 
    QgsVectorLayer, QgsFeature, QgsField, QgsFields,
    QgsPoint, QgsCoordinateReferenceSystem
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
        
        # Add to Vector Menu (under the main Plugins menu)
        self.iface.addPluginToVectorMenu("SHYFEM Tools", self.export_action)
        self.iface.addPluginToVectorMenu("SHYFEM Tools", self.import_action)
        
        # Add to Toolbar
        self.iface.addToolBarIcon(self.export_action)
        self.iface.addToolBarIcon(self.import_action)
        
        # Store actions for cleanup
        self.actions = [self.export_action, self.import_action]
        
    def unload(self):
        # Remove from menu and toolbar
        self.iface.removePluginVectorMenu("SHYFEM Tools", self.export_action)
        self.iface.removePluginVectorMenu("SHYFEM Tools", self.import_action)
        
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
                
                # Write line connections
                for line_id, point_ids in line_connections:
                    f.write(f"3 {line_id} 0 {len(point_ids)}\n")
                    f.write(" " + " ".join(map(str, point_ids)) + "\n")
            
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
        """Parse SHYFEM .grd file and extract points and lines"""
        points = {}  # {point_id: (x, y, point_type)}
        lines = []   # [(line_id, line_type, point_ids)]
        
        try:
            with open(filename, 'r') as f:
                content = f.read()
            
            lines_content = content.strip().split('\n')
            i = 0
            
            # Parse points (starting with '1')
            while i < len(lines_content):
                line = lines_content[i].strip()
                if not line:
                    i += 1
                    continue
                    
                parts = line.split()
                if len(parts) >= 5 and parts[0] == '1':
                    # Point line: 1 point_id point_type x y
                    point_id = int(parts[1])
                    point_type = int(parts[2])
                    x = float(parts[3])
                    y = float(parts[4])
                    points[point_id] = (x, y, point_type)
                    i += 1
                elif len(parts) >= 4 and parts[0] == '3':
                    # Line definition: 3 line_id line_type num_points
                    line_id = int(parts[1])
                    line_type = int(parts[2])
                    num_points = int(parts[3])
                    
                    # Next line should contain the point IDs
                    if i + 1 < len(lines_content):
                        point_line = lines_content[i + 1].strip()
                        point_ids = list(map(int, point_line.split()))
                        
                        if len(point_ids) == num_points:
                            lines.append((line_id, line_type, point_ids))
                        else:
                            print(f"Warning: Expected {num_points} points, got {len(point_ids)} for line {line_id}")
                        
                        i += 2  # Skip the points line
                    else:
                        i += 1
                else:
                    i += 1
                    
        except Exception as e:
            print(f"Error parsing SHYFEM file: {e}")
            return None, None
            
        return points, lines
    
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
        provider.addAttributes(fields)
        point_layer.updateFields()
        
        # Add features
        features = []
        for point_id, (x, y, point_type) in points.items():
            feature = QgsFeature()
            feature.setGeometry(QgsGeometry.fromPointXY(QgsPointXY(x, y)))
            feature.setAttributes([point_id, point_type, x, y])
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
                    x, y, _ = points_dict[pid]
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
    
    def import_shyfem_to_qgis(self, filename):
        """Main import function"""
        try:
            # Parse the GRD file
            points, lines = self.parse_shyfem_file(filename)
            
            if points is None or lines is None:
                return False
            
            if not points and not lines:
                QMessageBox.warning(self.iface.mainWindow(), "Error", 
                                  "No valid points or lines found in the GRD file")
                return False
            
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
            
            # Create line layer if we have lines
            if lines:
                line_layer = self.create_line_layer(lines, points, base_name)
                QgsProject.instance().addMapLayer(line_layer, False)
                group.addLayer(line_layer)
                layers_added.append(line_layer)
            
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
            
            print(f"Imported: {len(points)} points, {len(lines)} lines")
            return True
            
        except Exception as e:
            QMessageBox.critical(self.iface.mainWindow(), "Import Error", f"Error during import: {str(e)}")
            return False