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

#ifndef vtkTensorFieldCriticalCells_h
#define vtkTensorFieldCriticalCells_h

#include <vtkArrayDispatch.h>
#include <vtkBitArray.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkEdgeTable.h>
#include <vtkIntArray.h>
#include <vtkPointData.h>
#include <vtkPointSet.h>
#include <vtkPointSetAlgorithm.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTensorFieldCriticalCellsFilterModule.h>
#include <vtkTriangle.h>
#include <vtkUnsignedCharArray.h>

#include <array>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <variant>

class VTKTENSORFIELDCRITICALCELLSFILTER_EXPORT vtkTensorFieldCriticalCells
    : public vtkPointSetAlgorithm {
  public:
  using Dispatcher = vtkArrayDispatch::DispatchByValueType<vtkArrayDispatch::Reals>;

  using OutputPrecision = double;

  enum class EdgeRotation : vtkIdType { Positive = -1, Negative = 1, None = 0, Uninitialized = -2 };

  enum class EdgeValue : vtkIdType { Wide = -1, Perpendicular = 0, Narrow = 1, Uninitialized = -2 };

  template <typename T>
  struct SubdivisionData {
    SubdivisionData() : Coefficients(T{-1}), Anisotropy(T(-1)), NewVertexID(-1) {}

    std::variant<std::array<T, 2>, T> Coefficients;
    T Anisotropy;
    vtkIdType NewVertexID;
  };

  template <typename T>
  struct Triangle {
    Triangle() = delete;

    Triangle(std::array<vtkIdType, 3> vertexIDs, std::array<vtkIdType, 3> edgeIDs)
        : VertexIDs(vertexIDs), EdgeIDs(edgeIDs), Subdivision(std::nullopt) {}

    std::array<vtkIdType, 3> VertexIDs;
    std::array<vtkIdType, 3> EdgeIDs;
    std::optional<SubdivisionData<T>> Subdivision;
  };

  struct EdgeInformation {
    EdgeInformation()
        : ID(0)
        , Rotation(EdgeRotation::Uninitialized)
        , Value(EdgeValue::Uninitialized)
        , Subdivision(std::nullopt) {}

    EdgeInformation(EdgeRotation rotation, EdgeValue value)
        : ID(NumberOfEdges++), Rotation(rotation), Value(value), Subdivision(std::nullopt) {}

    vtkIdType ID;
    EdgeRotation Rotation;
    EdgeValue Value;
    std::optional<SubdivisionData<OutputPrecision>> Subdivision;

    static vtkIdType NumberOfEdges;
  };

  struct AlgorithmSetup {
    AlgorithmSetup()
        : InField(nullptr)
        , OutField(nullptr)
        , OutMesh(nullptr)
        , Eigenvectors(nullptr)
        , Tensors(nullptr)
        , DegenerateCellsFlags(nullptr)
        , DegenerateCellsTypes(nullptr)
        , EdgeTable(nullptr)
        , EdgeInformationVector({})
        , Triangles({}) {}

    AlgorithmSetup(vtkSmartPointer<vtkPointSet> inField, vtkSmartPointer<vtkPointSet> outField,
                   vtkSmartPointer<vtkPolyData> outMesh, vtkSmartPointer<vtkDataArray> eigenvectors,
                   vtkSmartPointer<vtkDataArray> tensors,
                   vtkSmartPointer<vtkBitArray> degenerateCellFlags,
                   vtkSmartPointer<vtkIntArray> degenerateCellTypes,
                   vtkSmartPointer<vtkEdgeTable> edgeTable)
        : InField(inField)
        , OutField(outField)
        , OutMesh(outMesh)
        , Eigenvectors(eigenvectors)
        , Tensors(tensors)
        , DegenerateCellsFlags(degenerateCellFlags)
        , DegenerateCellsTypes(degenerateCellTypes)
        , EdgeTable(edgeTable)
        , EdgeInformationVector({})
        , Triangles({}) {}

    vtkSmartPointer<vtkPointSet> InField;
    vtkSmartPointer<vtkPointSet> OutField;
    vtkSmartPointer<vtkPolyData> OutMesh;
    vtkSmartPointer<vtkDataArray> Eigenvectors;
    vtkSmartPointer<vtkDataArray> Tensors;
    vtkSmartPointer<vtkBitArray> DegenerateCellsFlags;
    vtkSmartPointer<vtkIntArray> DegenerateCellsTypes;
    vtkSmartPointer<vtkEdgeTable> EdgeTable;
    std::vector<std::shared_ptr<EdgeInformation>> EdgeInformationVector;
    std::vector<Triangle<OutputPrecision>> Triangles;
  };

  static vtkTensorFieldCriticalCells* New();

  vtkTypeMacro(vtkTensorFieldCriticalCells, vtkPointSetAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  vtkSetMacro(PerturbateIfNecessary, vtkTypeBool);
  vtkGetMacro(PerturbateIfNecessary, vtkTypeBool);
  vtkBooleanMacro(PerturbateIfNecessary, vtkTypeBool);

  vtkSetMacro(CopyEigenvectors, vtkTypeBool);
  vtkGetMacro(CopyEigenvectors, vtkTypeBool);
  vtkBooleanMacro(CopyEigenvectors, vtkTypeBool);

  vtkSetMacro(CopyTensors, vtkTypeBool);
  vtkGetMacro(CopyTensors, vtkTypeBool);
  vtkBooleanMacro(CopyTensors, vtkTypeBool);

  vtkSetMacro(LineField, std::string);
  vtkGetMacro(LineField, std::string);

  vtkSetMacro(TensorField, std::string);
  vtkGetMacro(TensorField, std::string);

  protected:
  vtkTensorFieldCriticalCells();
  int FillInputPortInformation(int port, vtkInformation* info) override;
  int FillOutputPortInformation(int port, vtkInformation* info) override;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

  private:
  vtkTensorFieldCriticalCells(const vtkTensorFieldCriticalCells&) = delete;
  void operator=(const vtkTensorFieldCriticalCells&) = delete;

  void Init(vtkInformationVector** inputVector, vtkInformationVector* outputVector);
  bool CheckInput() const;
  void BuildEdgeTable();
  bool CheckLineField() const;
  void PerturbateLineField();
  void ClassifyEdges();
  void ClassifyCells();
  bool SubdivideMesh();

  // name of the input array names.
  std::string LineField{};
  std::string TensorField{};

  // Algorithm properties
  vtkTypeBool PerturbateIfNecessary;
  vtkTypeBool CopyEigenvectors;
  vtkTypeBool CopyTensors;

  // Convenience members
  std::shared_ptr<AlgorithmSetup> Setup;
};

#endif  // vtkTensorFieldCriticalCells_h