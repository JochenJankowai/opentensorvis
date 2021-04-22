set(classes
  vtkTensorFieldCriticalCells)

vtk_module_add_module(OpenTensorVis::TensorFieldCriticalCells
  CLASSES ${classes})

paraview_add_server_manager_xmls(
  XMLS  TensorFieldCriticalCells.xml)
