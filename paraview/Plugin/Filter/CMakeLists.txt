set(classes
  vtkTensorFieldCriticalCells)

# set(headers
#   vtkTensorFieldCriticalCells.h)

# set(sources
#   vtkTensorFieldCriticalCells.cpp)

vtk_module_add_module(TensorFieldCriticalCells::TensorFieldCriticalCellsFilter
  CLASSES ${classes})

paraview_add_server_manager_xmls(
  XMLS  TensorFieldCriticalCellsFilter.xml)
