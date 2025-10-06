def classFactory(iface):
    from .shyfem_tools import ShyfemTools
    return ShyfemTools(iface)