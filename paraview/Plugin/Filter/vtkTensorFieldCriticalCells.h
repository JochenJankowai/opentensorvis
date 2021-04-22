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

#include <OpenTensorVisModule.h> // For export macro
#include <iterator>
#include <unordered_set>
#include <vtkArrayDispatch.h>
#include <vtkEdgeTable.h>
#include <vtkIntArray.h>
#include <vtkPointSetAlgorithm.h>
#include <vtkSmartPointer.h>
#include <vtkPointSet.h>
#include <string>

class OPENTENSORVIS_EXPORT vtkTensorFieldCriticalCells
    : public vtkPointSetAlgorithm {
public:
  enum class EdgeRotation : vtkIdType {
    Positive = -1,
    Negative = 1,
    None = 0,
    Uninitialized = -2
  };

  enum class EdgeValue : vtkIdType {
    Wide = -1,
    Perpendicular = 0,
    Narrow = 1,
    Uninitialized = -2
  };

  struct EdgeInformation {
    EdgeInformation()
        : Rotation(EdgeRotation::Uninitialized),
          Value(EdgeValue::Uninitialized) {}

    EdgeInformation(const EdgeRotation rotation, const EdgeValue value)
        : Rotation(rotation), Value(value) {}

    EdgeRotation Rotation;
    EdgeValue Value;
  };

  struct AlgorithmSetup {
    AlgorithmSetup()
        : InField(nullptr), OutField(nullptr), Eigenvectors(nullptr),
          Tensors(nullptr), DegenerateCells(nullptr), EdgeTable(nullptr) {}

    AlgorithmSetup(vtkSmartPointer<vtkPointSet> inField,
                   vtkSmartPointer<vtkPointSet> outField,
                   vtkSmartPointer<vtkDataArray> eigenvectors,
                   vtkSmartPointer<vtkDataArray> tensors,
                   vtkSmartPointer<vtkIntArray> degenerateCellFlags,
                   vtkSmartPointer<vtkEdgeTable> edgeTable)
        : InField(inField), OutField(outField), Eigenvectors(eigenvectors),
          Tensors(tensors), DegenerateCells(degenerateCellFlags),
          EdgeTable(edgeTable) {}

    vtkSmartPointer<vtkPointSet> InField;
    vtkSmartPointer<vtkPointSet> OutField;
    vtkSmartPointer<vtkDataArray> Eigenvectors;
    vtkSmartPointer<vtkDataArray> Tensors;
    vtkSmartPointer<vtkIntArray> DegenerateCells;
    vtkSmartPointer<vtkEdgeTable> EdgeTable;
    std::vector<std::shared_ptr<EdgeInformation>> EdgeInformationVector;
  };

  using Dispatcher =
      vtkArrayDispatch::DispatchByValueType<vtkArrayDispatch::Reals>;

  static vtkTensorFieldCriticalCells *New();

  vtkTypeMacro(vtkTensorFieldCriticalCells, vtkPointSetAlgorithm);
  void PrintSelf(ostream &os, vtkIndent indent) override;

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
  int FillInputPortInformation(int port, vtkInformation *info) override;
  int RequestData(vtkInformation *, vtkInformationVector **,
                  vtkInformationVector *) override;

private:
  vtkTensorFieldCriticalCells(const vtkTensorFieldCriticalCells &) = delete;
  void operator=(const vtkTensorFieldCriticalCells &) = delete;

  void Init(vtkInformationVector **inputVector,
            vtkInformationVector *outputVector);
  bool CheckInput() const;
  void BuildEdgeTable();
  bool CheckLineField() const;
  void PerturbateLineField();
  void ClassifyCells();

  // name of the input array names.
  std::string LineField;
  std::string TensorField;

  // Algorithm properties
  vtkTypeBool PerturbateIfNecessary;
  vtkTypeBool CopyEigenvectors;
  vtkTypeBool CopyTensors;

  // Convenience members
  std::shared_ptr<AlgorithmSetup> Setup;
};

#endif // vtkTensorFieldCriticalCells_h
