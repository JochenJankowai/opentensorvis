/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPolyDataAlgorithm.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkTensorFieldCriticalCells.h"

#include "vtkCell.h"
#include "vtkDoubleArray.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkStructuredGrid.h"
#include <array>
#include <unordered_set>
#include <vtkBitArray.h>
#include <vtkCellData.h>
#include <vtkCellTypes.h>
#include <vtkDelaunay2D.h>
#include <vtkIdList.h>
#include <vtkPointData.h>

vtkStandardNewMacro(vtkTensorFieldCriticalCells);

//----------------------------------------------------------------------------

vtkTensorFieldCriticalCells::vtkTensorFieldCriticalCells()
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(2);
  this->PerturbateIfNecessaryOn();
  this->CopyEigenvectorsOn();
  this->CopyTensorsOn();
}

//----------------------------------------------------------------------------

void vtkTensorFieldCriticalCells::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------

int vtkTensorFieldCriticalCells::FillInputPortInformation(int, vtkInformation* info)
{
  // port expects a uniform grid containing an array defining the line field
  info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPointSet");
  return 1;
}

//----------------------------------------------------------------------------

int vtkTensorFieldCriticalCells::FillOutputPortInformation(int port, vtkInformation* info)
{
  if (port == 0)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPointSet");
  } 
  else if (port == 1)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData");
  } 
  return 1;
}

//----------------------------------------------------------------------------

namespace
{
struct EdgeTableBuildFunctor
{
  EdgeTableBuildFunctor() = delete;
  EdgeTableBuildFunctor(std::shared_ptr<vtkTensorFieldCriticalCells::AlgorithmSetup> setup)
    : Setup(setup)
  {
  }

  template<typename EigenVectorArray>
  void operator()(EigenVectorArray* eigenvectors)
  {
    using ValueType = vtk::GetAPIType<EigenVectorArray>;

    vtkDataArrayAccessor<EigenVectorArray> eigenvectorsAccessor(eigenvectors);

    const auto numberOfCells = Setup->InField->GetNumberOfCells();
    const auto numberOfPoints = Setup->InField->GetNumberOfPoints();

    // According to Euler-Poincare, E = V + F - 2 (assuming genus of 0 and # of
    // shells := 1)
    Setup->EdgeTable->InitEdgeInsertion(numberOfPoints, 2);

    auto edgeValueFn = [this](const ValueType* ev1,
                         const ValueType* ev2) -> vtkTensorFieldCriticalCells::EdgeValue {
      const auto dotProduct = vtkMath::Dot2D(ev1, ev2);

      if (std::abs(dotProduct) < std::numeric_limits<ValueType>::epsilon())
      {
        return vtkTensorFieldCriticalCells::EdgeValue::Perpendicular;
      }

      return vtkTensorFieldCriticalCells::EdgeValue(
        vtkIdType(dotProduct > ValueType{ 0 }) - vtkIdType(dotProduct < ValueType{ 0 }));
    };

    auto pointIDs = vtkSmartPointer<vtkIdList>::New();

    for (vtkIdType cellIndex{ 0 }; cellIndex < numberOfCells; ++cellIndex)
    {
      pointIDs->Resize(0);
      Setup->InField->GetCellPoints(cellIndex, pointIDs);

      for (vtkIdType i{ 0 }; i < 3; ++i)
      {
        const auto p1 = pointIDs->GetId(i);
        const auto p2 = pointIDs->GetId((i + 1) % 3);

        ValueType ev1[2];
        ValueType ev2[2];

        void* edgeInfoPtr{ nullptr };

        Setup->EdgeTable->IsEdge(p1, p2, edgeInfoPtr);

        // Insert unique edge, if not present
        if (!edgeInfoPtr)
        {
          eigenvectorsAccessor.Get(p1, ev1);
          eigenvectorsAccessor.Get(p2, ev2);

          const auto edgeValue = edgeValueFn(ev1, ev2);

          auto edgeInformation = std::make_shared<vtkTensorFieldCriticalCells::EdgeInformation>(
            vtkTensorFieldCriticalCells::EdgeRotation::Uninitialized, edgeValue);

          Setup->EdgeInformationVector.push_back(edgeInformation);

          Setup->EdgeTable->InsertEdge(p1, p2, Setup->EdgeInformationVector.back().get());
        }
      }
    }
  }

private:
  std::shared_ptr<vtkTensorFieldCriticalCells::AlgorithmSetup> Setup;
};

struct LineFieldPerturbationFunctor
{
  LineFieldPerturbationFunctor() = delete;
  LineFieldPerturbationFunctor(std::shared_ptr<vtkTensorFieldCriticalCells::AlgorithmSetup> setup)
    : Setup(setup)
  {
  }

  template<typename EigenVectorArray>
  void operator()(EigenVectorArray* eigenvectors)
  {
    using ValueType = vtk::GetAPIType<EigenVectorArray>;

    vtkDataArrayAccessor<EigenVectorArray> eigenvectorsAccessor(eigenvectors);

    auto edgeValueFn = [this](const ValueType* ev1,
                         const ValueType* ev2) -> vtkTensorFieldCriticalCells::EdgeValue {
      const auto dotProduct = vtkMath::Dot2D(ev1, ev2);

      if (std::abs(dotProduct) < std::numeric_limits<ValueType>::epsilon())
      {
        return vtkTensorFieldCriticalCells::EdgeValue::Perpendicular;
      }

      return vtkTensorFieldCriticalCells::EdgeValue(
        vtkIdType(dotProduct > ValueType{ 0 }) - vtkIdType(dotProduct < ValueType{ 0 }));
    };

    vtkIdType p1{ 0 };
    vtkIdType p2{ 0 };

    void* edgeInfoPtr{ nullptr };

    Setup->EdgeTable->InitTraversal();

    while (Setup->EdgeTable->GetNextEdge(p1, p2, edgeInfoPtr))
    {
      auto edgeInfo = static_cast<vtkTensorFieldCriticalCells::EdgeInformation*>(edgeInfoPtr);
      if (edgeInfo->Value == vtkTensorFieldCriticalCells::EdgeValue::Perpendicular)
      {
        ValueType ev1[2];
        // Perturbate (rotate ever so slightly)
        eigenvectorsAccessor.Get(p1, ev1);
        const auto length = vtkMath::Norm2D(ev1);
        ev1[0] += std::numeric_limits<ValueType>::epsilon();

        // Preserve lenght
        vtkMath::Normalize2D(ev1);
        vtkMath::MultiplyScalar2D(ev1, length);

        // Set new vector (not sure this is necessary since we are working with
        // pointers.
        eigenvectorsAccessor.Set(p1, ev1);

        break;
      }
    }

    // vtkEdgeTable does not have a method that returns all edges connected to a
    // point. So here goes a semi-efficient way of updating all edges that
    // contain p1.
    const auto numberOfPoints = Setup->Eigenvectors->GetNumberOfTuples();
    for (auto p2{ 0 }; p2 < numberOfPoints; ++p2)
    {
      void* edgeInfoPtr{ nullptr };
      Setup->EdgeTable->IsEdge(p1, p2, edgeInfoPtr);
      if (edgeInfoPtr)
      {
        auto edgeInfo = static_cast<vtkTensorFieldCriticalCells::EdgeInformation*>(edgeInfoPtr);
        ValueType ev1[2];
        ValueType ev2[2];

        eigenvectorsAccessor.Get(p1, ev1);
        eigenvectorsAccessor.Get(p2, ev2);

        const auto edgeValue = edgeValueFn(ev1, ev2);

        edgeInfo->Value = edgeValue;
      }
    }
  }

private:
  std::shared_ptr<vtkTensorFieldCriticalCells::AlgorithmSetup> Setup;
};

struct EdgeClassificationFunctor
{
  EdgeClassificationFunctor() = delete;
  EdgeClassificationFunctor(std::shared_ptr<vtkTensorFieldCriticalCells::AlgorithmSetup> setup)
    : Setup(setup)
  {
  }

  template<typename TensorArray>
  void operator()(TensorArray* tensors)
  {
    using ValueType = vtk::GetAPIType<TensorArray>;

    vtkDataArrayAccessor<TensorArray> tensorsAccessor(tensors);

    auto signum = [](ValueType val) { return (ValueType{ 0 } < val) - (val < ValueType{ 0 }); };

    auto edgeRotationFn = [this, signum](const ValueType* tensor1,
                            const ValueType* tensor2) -> vtkTensorFieldCriticalCells::EdgeRotation {
      auto delta1 = (tensor1[0] - tensor1[3]) / ValueType{ 2 };
      auto delta2 = (tensor2[0] - tensor2[3]) / ValueType{ 2 };

      auto c1 = tensor1[1];
      auto c2 = tensor2[1];

      auto sgn = signum(c2 * delta1 - c1 * delta2);

      return vtkTensorFieldCriticalCells::EdgeRotation(sgn);
    };

    vtkIdType p1{ 0 };
    vtkIdType p2{ 0 };

    ValueType tensor1[4];
    ValueType tensor2[4];

    void* edgeInfoPtr{ nullptr };

    Setup->EdgeTable->InitTraversal();

    while (Setup->EdgeTable->GetNextEdge(p1, p2, edgeInfoPtr))
    {
      tensorsAccessor.Get(p1, tensor1);
      tensorsAccessor.Get(p2, tensor2);

      auto rotation = edgeRotationFn(tensor1, tensor2);

      auto edgeInfo = static_cast<vtkTensorFieldCriticalCells::EdgeInformation*>(edgeInfoPtr);

      edgeInfo->Rotation = rotation;
    }
  }

private:
  std::shared_ptr<vtkTensorFieldCriticalCells::AlgorithmSetup> Setup;
};
} // namespace

//----------------------------------------------------------------------------

int vtkTensorFieldCriticalCells::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  //----------------------------------------------------------------------------
  // Initialize algorithm. Fetch inport data, etc.
  //----------------------------------------------------------------------------
  Init(inputVector, outputVector);

  //----------------------------------------------------------------------------
  // Check input.
  //----------------------------------------------------------------------------
  if (!CheckInput())
    return 1;

  //----------------------------------------------------------------------------
  // Check cell types and perform triangulation if desired and necessary.
  //----------------------------------------------------------------------------
  auto cellTypes = vtkSmartPointer<vtkCellTypes>::New();
  Setup->InField->GetCellTypes(cellTypes);
  const auto numberOfCellTypes = cellTypes->GetNumberOfTypes();

  const auto oneCellTypeButNotTriangles =
    (numberOfCellTypes == 1 && !cellTypes->IsType(VTK_TRIANGLE));
  const auto moreThanOneCellType = (numberOfCellTypes != 1);

  if (oneCellTypeButNotTriangles || moreThanOneCellType)
  {
    vtkErrorMacro("Cell types not either uniform or triangles, terminating.");
    return 1;
  }

  //----------------------------------------------------------------------------
  // Build initial edge table
  //----------------------------------------------------------------------------
  BuildEdgeTable();

  //----------------------------------------------------------------------------
  // Check line field for perpendicular vectors and perturbate if desired and
  // necessary. Update edge table afterwards. Repeat until line field is valid.
  //----------------------------------------------------------------------------
  if (PerturbateIfNecessary)
  {
    while (!CheckLineField())
    {
      PerturbateLineField();
    }
  }
  else
  {
    vtkErrorMacro("Line field contains perpendicular vectors but perturbation "
                  "is not desired. Terminating.");
    return 1;
  }

  //----------------------------------------------------------------------------
  // Classify edges.
  //----------------------------------------------------------------------------
  ClassifyEdges();

  //----------------------------------------------------------------------------
  // Classify cells.
  //----------------------------------------------------------------------------
  ClassifyCells();

  //----------------------------------------------------------------------------
  // Add result to output data.
  //----------------------------------------------------------------------------
  std::cout << "Number of cells: " << std::to_string(Setup->OutField->GetNumberOfCells())
            << std::endl;

  auto outCellData = Setup->OutField->GetCellData();

  outCellData->AddArray(Setup->DegenerateCellsFlags);
  outCellData->AddArray(Setup->DegenerateCellsTypes);
  outCellData->Update();

  if (CopyEigenvectors)
  {
    auto outPointData = Setup->OutField->GetPointData();
    outPointData->AddArray(Setup->Eigenvectors);
    outPointData->Update();
  }

  if (CopyTensors)
  {
    auto outPointData = Setup->OutField->GetPointData();
    outPointData->AddArray(Setup->Tensors);
    outPointData->Update();
  }

  auto outportData2 = outputVector->GetInformationObject(1);
  vtkSmartPointer<vtkPolydata> outputMesh =
    vtkPolyData::SafeDownCast(outportData->Get(vtkDataObject::DATA_OBJECT()));

  SubdivideMesh(outputMesh, Setup->InField, Setup->Tensors);

  return 1;
}

//----------------------------------------------------------------------------

void vtkTensorFieldCriticalCells::Init(
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  //----------------------------------------------------------------------------
  // Obtain the input/output port info and data.
  //----------------------------------------------------------------------------
  auto inportData = inputVector[0]->GetInformationObject(0);
  auto outportData = outputVector->GetInformationObject(0);

  vtkSmartPointer<vtkPointSet> inField =
    vtkPointSet::SafeDownCast(inportData->Get(vtkDataObject::DATA_OBJECT()));
  vtkSmartPointer<vtkPointSet> outField =
    vtkPointSet::SafeDownCast(outportData->Get(vtkDataObject::DATA_OBJECT()));

  outField->CopyStructure(inField);

  vtkSmartPointer<vtkPointData> inPointData = inField->GetPointData();

  auto degenerateCellFlags = vtkSmartPointer<vtkBitArray>::New();
  degenerateCellFlags->SetNumberOfComponents(1);
  degenerateCellFlags->Allocate(inField->GetNumberOfCells());
  degenerateCellFlags->SetComponentName(0, "Flag value");
  degenerateCellFlags->SetName("Degenerate cell flags");

  auto degenerateCellTypes = vtkSmartPointer<vtkIntArray>::New();
  degenerateCellTypes->SetNumberOfComponents(1);
  degenerateCellTypes->Allocate(inField->GetNumberOfCells());
  degenerateCellTypes->SetComponentName(0, "Type value");
  degenerateCellTypes->SetName("Degenerate cell types");

  vtkSmartPointer<vtkDataArray> eigenvectors = inPointData->GetArray(LineField.c_str());
  vtkSmartPointer<vtkDataArray> tensors = inPointData->GetArray(TensorField.c_str());

  //----------------------------------------------------------------------------
  // Bundle everything up for easy access
  //----------------------------------------------------------------------------
  Setup = std::make_shared<AlgorithmSetup>(inField, outField, eigenvectors, tensors,
    degenerateCellFlags, degenerateCellTypes, vtkSmartPointer<vtkEdgeTable>::New());
}

//----------------------------------------------------------------------------

bool vtkTensorFieldCriticalCells::CheckInput() const
{
  auto dataType = Setup->Eigenvectors->GetDataType();
  if (dataType != VTK_FLOAT && dataType != VTK_DOUBLE)
  {
    vtkErrorMacro("Input data type for eigenvectors expected to be either "
                  "single or double precision floating "
                  "point. Input data type is "
      << Setup->Eigenvectors->GetDataTypeAsString());
    return false;
  }

  dataType = Setup->Tensors->GetDataType();
  if (dataType != VTK_FLOAT && dataType != VTK_DOUBLE)
  {
    vtkErrorMacro("Input data type for tensors expected to be either single or "
                  "double precision floating "
                  "point. Input data type is "
      << Setup->Tensors->GetDataTypeAsString());
    return false;
  }

  if (Setup->Eigenvectors->GetNumberOfComponents() != 2)
  {
    vtkErrorMacro("The specified line field does not contain 2D vectors.");
    return false;
  }

  if (Setup->Tensors->GetNumberOfComponents() != 4)
  {
    vtkErrorMacro("The specified tensor field does not contain 2D tensors.");
    return false;
  }

  return true;
}

//----------------------------------------------------------------------------

void vtkTensorFieldCriticalCells::BuildEdgeTable()
{
  EdgeTableBuildFunctor edgeTableBuilder(Setup);

  if (!Dispatcher::Execute(Setup->Eigenvectors, edgeTableBuilder))
  {
    edgeTableBuilder(Setup->Eigenvectors.GetPointer());
  }
}

//----------------------------------------------------------------------------

bool vtkTensorFieldCriticalCells::CheckLineField() const
{
  vtkIdType p1{ 0 };
  vtkIdType p2{ 0 };

  void* edgeInfoPtr{ nullptr };

  Setup->EdgeTable->InitTraversal();

  while (Setup->EdgeTable->GetNextEdge(p1, p2, edgeInfoPtr))
  {
    auto edgeInfo = static_cast<EdgeInformation*>(edgeInfoPtr);
    if (edgeInfo->Value == EdgeValue::Perpendicular)
    {
      return false;
    }
  }

  return true;
}

//----------------------------------------------------------------------------

void vtkTensorFieldCriticalCells::PerturbateLineField()
{
  LineFieldPerturbationFunctor lineFieldPerturbator(Setup);

  if (!Dispatcher::Execute(Setup->Eigenvectors, lineFieldPerturbator))
  {
    lineFieldPerturbator(Setup->Eigenvectors.GetPointer());
  }
}

//----------------------------------------------------------------------------

void vtkTensorFieldCriticalCells::ClassifyEdges()
{
  EdgeClassificationFunctor edgeClassificationFunctor(Setup);

  if (!Dispatcher::Execute(Setup->Tensors, edgeClassificationFunctor))
  {
    edgeClassificationFunctor(Setup->Tensors.GetPointer());
  }
}

//----------------------------------------------------------------------------

void vtkTensorFieldCriticalCells::ClassifyCells()
{
  const auto numberOfCells = Setup->InField->GetNumberOfCells();

  auto pointIDs = vtkSmartPointer<vtkIdList>::New();

  for (vtkIdType cellIndex{ 0 }; cellIndex < numberOfCells; ++cellIndex)
  {
    pointIDs->Resize(0);
    Setup->InField->GetCellPoints(cellIndex, pointIDs);

    vtkIdType cellValue{ 1 };

    std::array<vtkTensorFieldCriticalCells::EdgeInformation*, 3> edgeInfos{ nullptr, nullptr,
      nullptr };

    for (vtkIdType i{ 0 }; i < 3; ++i)
    {
      const auto p1 = pointIDs->GetId(i);
      const auto p2 = pointIDs->GetId((i + 1) % 3);

      void* edgeInfoPtr{ nullptr };
      Setup->EdgeTable->IsEdge(p1, p2, edgeInfoPtr);
      if (edgeInfoPtr)
      {
        edgeInfos[i] = static_cast<vtkTensorFieldCriticalCells::EdgeInformation*>(edgeInfoPtr);

        cellValue *= static_cast<vtkIdType>(edgeInfos[i]->Value);
      }
    }

    if (cellValue < 0)
    {
      Setup->DegenerateCellsFlags->InsertNextValue(static_cast<int>(1));

      EdgeRotation direction{ EdgeRotation::Uninitialized };

      for (auto edgeInfo : edgeInfos)
      {
        direction = edgeInfo->Rotation;
        if (direction != EdgeRotation::None)
          break;
      }

      if (direction == EdgeRotation::None)
        vtkErrorMacro(<< "Degenerate cell without rotation. Result inconsistent.");

      Setup->DegenerateCellsTypes->InsertNextValue(static_cast<int>(direction));
    }
    else
    {
      Setup->DegenerateCellsFlags->InsertNextValue(static_cast<int>(0));
      Setup->DegenerateCellsTypes->InsertNextValue(static_cast<int>(EdgeRotation::None));
    }
  }
}

void vtkTensorFieldCriticalCells::addOutputTriangle(vtkCellArray *trianglesArr, 
  vtkTriangle *triangleVtk, vtkIdType p1, vtkIdType p2, vtkIdType p3)
{
  triangleVtk->GetPointIds()->SetId(0, p1);
  triangleVtk->GetPointIds()->SetId(1, p2);
  triangleVtk->GetPointIds()->SetId(2, p3);
  trianglesArr->InsertNextCell(triangleVtk);
}
