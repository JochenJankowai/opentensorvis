paraview_add_plugin(TensorFieldCriticalCells
  VERSION "1.0"
  MODULES TensorFieldCriticalCells::TensorFieldCriticalCellsFilter
  MODULE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/Filter/vtk.module")
