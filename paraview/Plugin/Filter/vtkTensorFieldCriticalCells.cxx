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

#include <vtkBitArray.h>
#include <vtkCell.h>
#include <vtkCellData.h>
#include <vtkCellTypes.h>
#include <vtkDelaunay2D.h>
#include <vtkDoubleArray.h>
#include <vtkIdList.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkStructuredGrid.h>
#include <vtkTensorFieldCriticalCells.h>

#include <array>
#include <cmath>
#include <unordered_set>

vtkStandardNewMacro(vtkTensorFieldCriticalCells);

//----------------------------------------------------------------------------

vtkTensorFieldCriticalCells::vtkTensorFieldCriticalCells() {
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(2);
  this->PerturbateIfNecessaryOn();
  this->CopyEigenvectorsOn();
  this->CopyTensorsOn();
}

//----------------------------------------------------------------------------

void vtkTensorFieldCriticalCells::PrintSelf(ostream& os, vtkIndent indent) {
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------

int vtkTensorFieldCriticalCells::FillInputPortInformation(int, vtkInformation* info) {
  // port expects a uniform grid containing an array defining the line field
  info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPointSet");
  return 1;
}

//----------------------------------------------------------------------------

int vtkTensorFieldCriticalCells::FillOutputPortInformation(int port, vtkInformation* info) {
  if (port == 0) {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPointSet");
  } else if (port == 1) {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData");
  }
  return 1;
}

//----------------------------------------------------------------------------

namespace {
struct EdgeTableBuildFunctor {
  EdgeTableBuildFunctor() = delete;
  EdgeTableBuildFunctor(std::shared_ptr<vtkTensorFieldCriticalCells::AlgorithmSetup> setup) : Setup(setup) {}

  template <typename EigenVectorArray>
  void operator()(EigenVectorArray* eigenvectors) {
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

      if (std::abs(dotProduct) < std::numeric_limits<ValueType>::epsilon()) {
        return vtkTensorFieldCriticalCells::EdgeValue::Perpendicular;
      }

      return vtkTensorFieldCriticalCells::EdgeValue(vtkIdType(dotProduct > ValueType{0}) -
                                                    vtkIdType(dotProduct < ValueType{0}));
    };

    auto pointIDs = vtkSmartPointer<vtkIdList>::New();

    for (vtkIdType cellIndex{0}; cellIndex < numberOfCells; ++cellIndex) {
      pointIDs->Resize(0);
      Setup->InField->GetCellPoints(cellIndex, pointIDs);

      std::array<vtkIdType, 3> vertexIDs{};

      for (vtkIdType i{0}; i < 3; ++i) {
        const auto p1 = pointIDs->GetId(i);
        const auto p2 = pointIDs->GetId((i + 1) % 3);

        vertexIDs[i] = p1;

        ValueType ev1[2];
        ValueType ev2[2];

        void* edgeInfoPtr{nullptr};

        Setup->EdgeTable->IsEdge(p1, p2, edgeInfoPtr);

        // Insert unique edge, if not present
        if (!edgeInfoPtr) {
          eigenvectorsAccessor.Get(p1, ev1);
          eigenvectorsAccessor.Get(p2, ev2);

          const auto edgeValue = edgeValueFn(ev1, ev2);

          auto edgeInformation = std::make_shared<vtkTensorFieldCriticalCells::EdgeInformation>(
              vtkTensorFieldCriticalCells::EdgeRotation::Uninitialized, edgeValue, cellIndex);

          Setup->EdgeInformationVector.push_back(edgeInformation);

          Setup->EdgeTable->InsertEdge(p1, p2, Setup->EdgeInformationVector.back().get());
        } else {
          auto edgeInformation = static_cast<vtkTensorFieldCriticalCells::EdgeInformation*>(edgeInfoPtr);
          edgeInformation->AdjacentCellIDs.push_back(cellIndex);
        }
      }

      Setup->Triangles.emplace_back(vertexIDs);
    }
  }

  private:
  std::shared_ptr<vtkTensorFieldCriticalCells::AlgorithmSetup> Setup;
};

struct LineFieldPerturbationFunctor {
  LineFieldPerturbationFunctor() = delete;
  LineFieldPerturbationFunctor(std::shared_ptr<vtkTensorFieldCriticalCells::AlgorithmSetup> setup)
      : Setup(setup) {}

  template <typename EigenVectorArray>
  void operator()(EigenVectorArray* eigenvectors) {
    using ValueType = vtk::GetAPIType<EigenVectorArray>;

    vtkDataArrayAccessor<EigenVectorArray> eigenvectorsAccessor(eigenvectors);

    auto edgeValueFn = [this](const ValueType* ev1,
                              const ValueType* ev2) -> vtkTensorFieldCriticalCells::EdgeValue {
      const auto dotProduct = vtkMath::Dot2D(ev1, ev2);

      if (std::abs(dotProduct) < std::numeric_limits<ValueType>::epsilon()) {
        return vtkTensorFieldCriticalCells::EdgeValue::Perpendicular;
      }

      return vtkTensorFieldCriticalCells::EdgeValue(vtkIdType(dotProduct > ValueType{0}) -
                                                    vtkIdType(dotProduct < ValueType{0}));
    };

    auto rotate2D = [](ValueType* vector, const ValueType angle) -> void {
      const ValueType Cos{std::cos(angle)};
      const ValueType Sin{std::sin(angle)};

      vector[0] = vector[0] * Cos - vector[1] * Sin;
      vector[1] = vector[0] * Sin + vector[1] * Cos;
    };

    vtkIdType p1{0};
    vtkIdType p2{0};

    void* edgeInfoPtr{nullptr};

    Setup->EdgeTable->InitTraversal();

    while (Setup->EdgeTable->GetNextEdge(p1, p2, edgeInfoPtr)) {
      auto edgeInfo = static_cast<vtkTensorFieldCriticalCells::EdgeInformation*>(edgeInfoPtr);
      if (edgeInfo->Value == vtkTensorFieldCriticalCells::EdgeValue::Perpendicular) {
        ValueType ev1[2];
        // Perturbate (rotate ever so slightly)
        eigenvectorsAccessor.Get(p1, ev1);

        constexpr auto angle = std::numeric_limits<ValueType>::epsilon();  // 0.0174533

        rotate2D(ev1, angle);

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
    for (p2 = 0; p2 < numberOfPoints; ++p2) {
      edgeInfoPtr = nullptr;
      Setup->EdgeTable->IsEdge(p1, p2, edgeInfoPtr);
      if (edgeInfoPtr) {
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

struct EdgeClassificationFunctor {
  EdgeClassificationFunctor() = delete;
  EdgeClassificationFunctor(std::shared_ptr<vtkTensorFieldCriticalCells::AlgorithmSetup> setup)
      : Setup(setup) {}

  template <typename TensorArray>
  void operator()(TensorArray* tensors) {
    using ValueType = vtk::GetAPIType<TensorArray>;

    vtkDataArrayAccessor<TensorArray> tensorsAccessor(tensors);

    auto signum = [](ValueType val) { return (ValueType{0} < val) - (val < ValueType{0}); };

    auto edgeRotationFn = [this, signum](
                              const ValueType* tensor1,
                              const ValueType* tensor2) -> vtkTensorFieldCriticalCells::EdgeRotation {
      auto delta1 = (tensor1[0] - tensor1[3]) / ValueType{2};
      auto delta2 = (tensor2[0] - tensor2[3]) / ValueType{2};

      auto c1 = tensor1[1];
      auto c2 = tensor2[1];

      auto sgn = signum(c2 * delta1 - c1 * delta2);

      return vtkTensorFieldCriticalCells::EdgeRotation(sgn);
    };

    vtkIdType p1{0};
    vtkIdType p2{0};

    ValueType tensor1[4];
    ValueType tensor2[4];

    void* edgeInfoPtr{nullptr};

    Setup->EdgeTable->InitTraversal();

    while (Setup->EdgeTable->GetNextEdge(p1, p2, edgeInfoPtr)) {
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

struct MeshSubdivisionFunctor {
  MeshSubdivisionFunctor() = delete;
  MeshSubdivisionFunctor(std::shared_ptr<vtkTensorFieldCriticalCells::AlgorithmSetup> setup)
      : Setup(setup), Success(false) {}

  template <typename TensorArray>
  void operator()(TensorArray* tensors) {
    Success = false;

    using ValueType = vtk::GetAPIType<TensorArray>;
    using ArrayType = vtkAOSDataArrayTemplate<ValueType>;
    using Precision = vtkTensorFieldCriticalCells::OutputPrecision;

    const auto originalNumberOfPoints = Setup->InField->GetNumberOfPoints();
    auto newPoints = vtkSmartPointer<vtkPoints>::New();

    auto anisotropyArray = vtkSmartPointer<ArrayType>::New();
    anisotropyArray->SetNumberOfComponents(1);
    anisotropyArray->SetName("Anisotropy");

    auto outputTensorsArray = vtkSmartPointer<ArrayType>::New();
    outputTensorsArray->SetNumberOfComponents(4);
    outputTensorsArray->SetName("Tensors");
    outputTensorsArray->DeepCopy(tensors);

    auto pointTypeArray = vtkSmartPointer<vtkUnsignedCharArray>::New();
    pointTypeArray->SetNumberOfComponents(1);
    pointTypeArray->SetName("Point Type");

    vtkSmartPointer<vtkBitArray> degenerateCellFlags =
        vtkBitArray::SafeDownCast(Setup->OutField->GetCellData()->GetArray("Degenerate cell flags"));
    vtkSmartPointer<vtkIntArray> degenerateCellTypes =
        vtkIntArray::SafeDownCast(Setup->OutField->GetCellData()->GetArray("Degenerate cell types"));

    vtkDataArrayAccessor<TensorArray> tensorsAccessor(tensors);

    for (vtkIdType pointIndex{0}; pointIndex < originalNumberOfPoints; ++pointIndex) {
      ValueType tensor[4];
      tensorsAccessor.Get(pointIndex, tensor);

      const auto E = tensor[0];
      const auto F = tensor[1];
      const auto G = tensor[3];

      const auto anisotropy = std::sqrt((E - G) * (E - G) + ValueType{4} * F * F);
      anisotropyArray->InsertNextValue(anisotropy);

      auto coords = Setup->InField->GetPoints()->GetPoint(pointIndex);
      newPoints->InsertNextPoint(coords[0], coords[1], coords[2]);
      pointTypeArray->InsertNextValue(0);
    }

    Setup->EdgeTable->InitTraversal();

    vtkIdType edgePoints{0};

    vtkIdType p1{0};
    vtkIdType p2{0};

    void* edgeInfoPtr{nullptr};

    vtkIdType triPoints{0};
    vtkIdType triangleID{0};
    for (auto& triangle : Setup->Triangles) {
      const auto p1 = triangle.VertexIDs[0];
      const auto p2 = triangle.VertexIDs[1];
      const auto p3 = triangle.VertexIDs[2];

      ValueType tensor1[4];
      ValueType tensor2[4];
      ValueType tensor3[4];

      tensorsAccessor.Get(p1, tensor1);
      tensorsAccessor.Get(p2, tensor2);
      tensorsAccessor.Get(p3, tensor3);

      const auto E1 = tensor1[0];
      const auto F1 = tensor1[1];
      const auto G1 = tensor1[3];

      const auto E2 = tensor2[0];
      const auto F2 = tensor2[1];
      const auto G2 = tensor2[3];

      const auto E3 = tensor3[0];
      const auto F3 = tensor3[1];
      const auto G3 = tensor3[3];

      const auto Ex = E1 - E3;
      const auto Fx = F1 - F3;
      const auto Gx = G1 - G3;

      const auto Ey = E2 - E3;
      const auto Fy = F2 - F3;
      const auto Gy = G2 - G3;

      const auto Ec = E3;
      const auto Fc = F3;
      const auto Gc = G3;

      const auto A = (Ex - Gx) * (Ex - Gx) + ValueType{4} * Fx * Fx;
      const auto B = ValueType{2} * (Ex - Gx) * (Ey - Gy) + ValueType{8} * Fx * Fy;
      const auto C = (Ey - Gy) * (Ey - Gy) + ValueType{4} * Fy * Fy;
      const auto D = ValueType{2} * (Ex - Gx) * (Ec - Gc) + ValueType{8} * Fx * Fc;
      const auto E = ValueType{2} * (Ey - Gy) * (Ec - Gc) + ValueType{8} * Fy * Fc;

      const auto H = ValueType{4} * A * C - B * B;

      if (H != ValueType{0}) {
        const auto alpha = (ValueType{-2} * C * D + B * E) / H;
        const auto beta = (ValueType{-2} * A * E + B * D) / H;
        const auto gamma = ValueType{1} - alpha - beta;

        // CHECK IF THIS CELL HAS BEEN IDENTIFIED TO CONTAIN A DEGENERATE POINT
        if (Setup->DegenerateCellsFlags->GetValue(triangleID)) {

          if (alpha > ValueType{0} && beta > ValueType{0} && gamma > ValueType{0}) {

            auto s = vtkTensorFieldCriticalCells::SubdivisionData<Precision>();

            s.Anisotropy = ValueType{0};
            s.Coefficients = std::array<Precision, 2>{alpha, beta};

            anisotropyArray->InsertNextValue(s.Anisotropy);

            ValueType triTensor[4];

            triTensor[0] = alpha * E1 + beta * E2 + gamma * E3;
            triTensor[1] = triTensor[2] = alpha * F1 + beta * F2 + gamma * F3;
            triTensor[3] = alpha * G1 + beta * G2 + gamma * G3;

            outputTensorsArray->InsertNextTuple(triTensor);

            double coords1[3];
            double coords2[3];
            double coords3[3];

            newPoints->GetPoint(p1, coords1);
            newPoints->GetPoint(p2, coords2);
            newPoints->GetPoint(p3, coords3);

            double coordsTriPoint[3];
            for (size_t i{0}; i < 3; ++i) {
              coordsTriPoint[i] = alpha * coords1[i] + beta * coords2[i] + gamma * coords3[i];
            }

            newPoints->InsertNextPoint(coordsTriPoint);
            pointTypeArray->InsertNextValue(2);
            s.NewVertexID = originalNumberOfPoints + triPoints;
            ++triPoints;

            triangle.Subdivision = s;
          }
        }
      }

      triangleID++;
    }

    while (Setup->EdgeTable->GetNextEdge(p1, p2, edgeInfoPtr)) {
      ValueType tensor1[4];
      ValueType tensor2[4];

      tensorsAccessor.Get(p1, tensor1);
      tensorsAccessor.Get(p2, tensor2);

      const auto E1 = tensor1[0];
      const auto F1 = tensor1[1];
      const auto G1 = tensor1[3];

      const auto E2 = tensor2[0];
      const auto F2 = tensor2[1];
      const auto G2 = tensor2[3];

      const auto A = (E2 - E1 - G2 + G1) * (E2 - E1 - G2 + G1) + ValueType{4} * (F2 - F1) * (F2 - F1);
      const auto B = ValueType{2} * (E2 - E1 - G2 + G1) * (E1 - G1) + ValueType{8} * F1 * (F2 - F1);
      const auto C = (E1 - G1) * (E1 - G1) + ValueType{4} * F1 * F1;

      if (A != 0) {
        const auto t = -B / (ValueType{2} * A);

        if (t > ValueType{0} && t < ValueType{1}) {
          // THERE IS A MINUMUM ALONG THE EDGE
          // THE CHECK IF ADJACENT CELLS CONTAIN A DEGENERATE POINT AND HAVE NOT YET BEEN SUBDIVIDED IS
          // AMBIGUOUS; LEAVE AS IS

          auto edgeInfo = static_cast<vtkTensorFieldCriticalCells::EdgeInformation*>(edgeInfoPtr);
          auto s = vtkTensorFieldCriticalCells::SubdivisionData<Precision>();

          s.Coefficients = t;

          s.Anisotropy = std::sqrt(A * t * t + B * t + C);
          anisotropyArray->InsertNextValue(s.Anisotropy);

          ValueType edgetensor[4];
          edgetensor[0] = (ValueType{1} - t) * E1 + t * E2;
          edgetensor[1] = edgetensor[2] = (ValueType{1} - t) * F1 + t * F2;
          edgetensor[3] = (ValueType{1} - t) * G1 + t * G2;
          outputTensorsArray->InsertNextTuple(edgetensor);

          double coords1[3];
          double coords2[3];

          newPoints->GetPoint(p1, coords1);
          newPoints->GetPoint(p2, coords2);

          double coordsEdgePoint[3];
          for (size_t i{0}; i < 3; ++i) {
            coordsEdgePoint[i] = (1.0 - double(t)) * coords1[i] + double(t) * coords2[i];
          }

          newPoints->InsertNextPoint(coordsEdgePoint);
          pointTypeArray->InsertNextValue(1);
          s.NewVertexID = originalNumberOfPoints + edgePoints + triPoints;
          ++edgePoints;

          edgeInfo->Subdivision = s;
        }
      }
    }

    vtkSmartPointer<vtkCellArray> trianglesArr = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkTriangle> triangleVtk = vtkSmartPointer<vtkTriangle>::New();

    auto addOutputTriangle = [trianglesArr, triangleVtk](const vtkIdType p1, const vtkIdType p2,
                                                         const vtkIdType p3) -> void {
      triangleVtk->GetPointIds()->SetId(0, p1);
      triangleVtk->GetPointIds()->SetId(1, p2);
      triangleVtk->GetPointIds()->SetId(2, p3);
      trianglesArr->InsertNextCell(triangleVtk);
    };

    for (const auto& triangle : Setup->Triangles) {
      const auto p1 = triangle.VertexIDs[0];
      const auto p2 = triangle.VertexIDs[1];
      const auto p3 = triangle.VertexIDs[2];

      void* edge1InfoPtr{nullptr};
      void* edge2InfoPtr{nullptr};
      void* edge3InfoPtr{nullptr};

      Setup->EdgeTable->IsEdge(p1, p2, edge1InfoPtr);
      Setup->EdgeTable->IsEdge(p2, p3, edge2InfoPtr);
      Setup->EdgeTable->IsEdge(p3, p1, edge3InfoPtr);

      if (!edge1InfoPtr || !edge2InfoPtr || !edge3InfoPtr) {
        return;
      }

      const auto e1 = static_cast<vtkTensorFieldCriticalCells::EdgeInformation*>(edge1InfoPtr);
      const auto e2 = static_cast<vtkTensorFieldCriticalCells::EdgeInformation*>(edge2InfoPtr);
      const auto e3 = static_cast<vtkTensorFieldCriticalCells::EdgeInformation*>(edge3InfoPtr);

      vtkIdType p12{0}, p23{0}, p13{0}, p123{0};
      if (triangle.Subdivision) {

        p123 = triangle.Subdivision.value().NewVertexID;

        if (e1->Subdivision) {
          p12 = e1->Subdivision.value().NewVertexID;
          addOutputTriangle(p123, p1, p12);
          addOutputTriangle(p123, p12, p2);
        } else {
          addOutputTriangle(p123, p1, p2);
        }
        if (e2->Subdivision) {
          p23 = e2->Subdivision.value().NewVertexID;
          addOutputTriangle(p123, p2, p23);
          addOutputTriangle(p123, p23, p3);
        } else {
          addOutputTriangle(p123, p2, p3);
        }
        if (e3->Subdivision) {
          p13 = e3->Subdivision.value().NewVertexID;
          addOutputTriangle(p123, p3, p13);
          addOutputTriangle(p123, p13, p1);
        } else {
          addOutputTriangle(p123, p3, p1);
        }
      } else {
        int numCrits = 0;
        if (e1->Subdivision) {
          p12 = e1->Subdivision.value().NewVertexID;
          numCrits++;
        }
        if (e2->Subdivision) {
          p23 = e2->Subdivision.value().NewVertexID;
          numCrits++;
        }
        if (e3->Subdivision) {
          p13 = e3->Subdivision.value().NewVertexID;
          numCrits++;
        }
        switch (numCrits) {
          case 0:
            addOutputTriangle(p1, p2, p3);
            break;
          case 1:
            if (e1->Subdivision) {
              addOutputTriangle(p12, p3, p1);
              addOutputTriangle(p12, p3, p2);
            } else if (e2->Subdivision) {
              addOutputTriangle(p23, p1, p2);
              addOutputTriangle(p23, p1, p3);
            } else if (e3->Subdivision) {
              addOutputTriangle(p13, p2, p3);
              addOutputTriangle(p13, p2, p1);
            }
            break;
          case 2:
            if (!e1->Subdivision) {
              addOutputTriangle(p23, p13, p3);
              if (!e2->Subdivision || !e3->Subdivision) {
                return;
              }

              if (e2->Subdivision.value().Anisotropy < e3->Subdivision.value().Anisotropy) {
                addOutputTriangle(p23, p13, p1);
                addOutputTriangle(p23, p2, p1);
              } else {
                addOutputTriangle(p13, p23, p2);
                addOutputTriangle(p13, p1, p2);
              }
            } else if (!e2->Subdivision) {
              addOutputTriangle(p12, p13, p1);

              if (!e1->Subdivision || !e3->Subdivision) {
                return;
              }

              if (e1->Subdivision.value().Anisotropy < e3->Subdivision.value().Anisotropy) {
                addOutputTriangle(p12, p13, p3);
                addOutputTriangle(p12, p2, p3);
              } else {
                addOutputTriangle(p13, p12, p2);
                addOutputTriangle(p13, p3, p2);
              }
            } else if (!e3->Subdivision) {
              addOutputTriangle(p12, p23, p2);

              if (!e1->Subdivision || !e2->Subdivision) {
                return;
              }

              if (e1->Subdivision.value().Anisotropy < e2->Subdivision.value().Anisotropy) {
                addOutputTriangle(p12, p23, p3);
                addOutputTriangle(p12, p1, p3);
              } else {
                addOutputTriangle(p23, p12, p1);
                addOutputTriangle(p23, p3, p1);
              }
            }
            break;
          case 3:
            if (!e1->Subdivision || !e2->Subdivision || !e3->Subdivision) {
              return;
            }

            const auto val12 = e1->Subdivision.value().Anisotropy;
            const auto val23 = e2->Subdivision.value().Anisotropy;
            const auto val13 = e3->Subdivision.value().Anisotropy;

            if (val12 < val23 && val12 < val13) {
              addOutputTriangle(p12, p1, p13);
              addOutputTriangle(p12, p13, p3);
              addOutputTriangle(p12, p3, p23);
              addOutputTriangle(p12, p23, p2);
            } else if (val23 < val12 && val23 < val13) {
              addOutputTriangle(p23, p2, p12);
              addOutputTriangle(p23, p12, p1);
              addOutputTriangle(p23, p1, p13);
              addOutputTriangle(p23, p13, p3);
            } else {
              addOutputTriangle(p13, p3, p23);
              addOutputTriangle(p13, p23, p2);
              addOutputTriangle(p13, p2, p12);
              addOutputTriangle(p13, p12, p1);
            }
            break;
        }
      }
    }

    Setup->OutMesh->SetPoints(newPoints);
    Setup->OutMesh->SetPolys(trianglesArr);

    auto outPointData = Setup->OutMesh->GetPointData();

    outPointData->AddArray(anisotropyArray);
    outPointData->AddArray(outputTensorsArray);
    outPointData->AddArray(pointTypeArray);
    outPointData->Update();

    Success = true;
  }

  bool Success;

  private:
  std::shared_ptr<vtkTensorFieldCriticalCells::AlgorithmSetup> Setup;
};
}  // namespace

//----------------------------------------------------------------------------

int vtkTensorFieldCriticalCells::RequestData(vtkInformation* vtkNotUsed(request),
                                             vtkInformationVector** inputVector,
                                             vtkInformationVector* outputVector) {
  //----------------------------------------------------------------------------
  // Initialize algorithm. Fetch inport data, etc.
  //----------------------------------------------------------------------------
  Init(inputVector, outputVector);

  //----------------------------------------------------------------------------
  // Check input.
  //----------------------------------------------------------------------------
  if (!CheckInput()) return 0;

  //----------------------------------------------------------------------------
  // Check cell types and perform triangulation if desired and necessary.
  //----------------------------------------------------------------------------
  auto cellTypes = vtkSmartPointer<vtkCellTypes>::New();
  Setup->InField->GetCellTypes(cellTypes);
  const auto numberOfCellTypes = cellTypes->GetNumberOfTypes();

  const auto oneCellTypeButNotTriangles = (numberOfCellTypes == 1 && !cellTypes->IsType(VTK_TRIANGLE));
  const auto moreThanOneCellType = (numberOfCellTypes != 1);

  if (oneCellTypeButNotTriangles || moreThanOneCellType) {
    vtkErrorMacro("Cell types not either uniform or triangles, terminating.");
    return 0;
  }

  //----------------------------------------------------------------------------
  // Build initial edge table
  //----------------------------------------------------------------------------
  BuildEdgeTable();

  //----------------------------------------------------------------------------
  // Check line field for perpendicular vectors and perturbate if desired and
  // necessary. Update edge table afterwards. Repeat until line field is valid.
  //----------------------------------------------------------------------------
  if (PerturbateIfNecessary) {
    while (!CheckLineField()) {
      PerturbateLineField();
    }
  } else {
    vtkErrorMacro(
        "Line field contains perpendicular vectors but perturbation "
        "is not desired. Terminating.");
    return 0;
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
  std::cout << "Number of cells: " << std::to_string(Setup->OutField->GetNumberOfCells()) << std::endl;

  auto outCellData = Setup->OutField->GetCellData();

  outCellData->AddArray(Setup->DegenerateCellsFlags);
  outCellData->AddArray(Setup->DegenerateCellsTypes);
  outCellData->Update();

  if (CopyEigenvectors) {
    auto outPointData = Setup->OutField->GetPointData();
    outPointData->AddArray(Setup->Eigenvectors);
    outPointData->Update();
  }

  if (CopyTensors) {
    auto outPointData = Setup->OutField->GetPointData();
    outPointData->AddArray(Setup->Tensors);
    outPointData->Update();
  }

  if (!SubdivideMesh()) {
    vtkErrorMacro(<< "Mesh subdivision failed.");
    return 0;
  }

  return 1;
}

//----------------------------------------------------------------------------

void vtkTensorFieldCriticalCells::Init(vtkInformationVector** inputVector,
                                       vtkInformationVector* outputVector) {
  //----------------------------------------------------------------------------
  // Obtain the input/output port info and data.
  //----------------------------------------------------------------------------
  auto inportData = inputVector[0]->GetInformationObject(0);
  auto outportDataObject1 = outputVector->GetInformationObject(0);
  auto outportDataObject2 = outputVector->GetInformationObject(1);

  vtkSmartPointer<vtkPointSet> inField =
      vtkPointSet::SafeDownCast(inportData->Get(vtkDataObject::DATA_OBJECT()));
  vtkSmartPointer<vtkPointSet> outField =
      vtkPointSet::SafeDownCast(outportDataObject1->Get(vtkDataObject::DATA_OBJECT()));
  vtkSmartPointer<vtkPolyData> outMesh =
      vtkPolyData::SafeDownCast(outportDataObject2->Get(vtkDataObject::DATA_OBJECT()));

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
  Setup =
      std::make_shared<AlgorithmSetup>(inField, outField, outMesh, eigenvectors, tensors, degenerateCellFlags,
                                       degenerateCellTypes, vtkSmartPointer<vtkEdgeTable>::New());
}

//----------------------------------------------------------------------------

bool vtkTensorFieldCriticalCells::CheckInput() const {
  auto writeDataTypeWarning = [this](const std::string arrayName, const std::string dataType) {
    vtkErrorMacro("Input data type for " << arrayName
                                         << " expected to be either "
                                            "single or double precision floating "
                                            "point. Input data type is "
                                         << dataType << ".");
  };

  auto writeComponentWarning = [this](const std::string fieldName, const std::string dataType) {
    vtkErrorMacro("The specified " << fieldName << " field does not contain 2D " << dataType << ".");
  };

  auto dataType = Setup->Eigenvectors->GetDataType();
  if (dataType != VTK_FLOAT && dataType != VTK_DOUBLE) {
    writeDataTypeWarning("eigenvectors", Setup->Eigenvectors->GetDataTypeAsString());
    return false;
  }

  dataType = Setup->Tensors->GetDataType();
  if (dataType != VTK_FLOAT && dataType != VTK_DOUBLE) {
    writeDataTypeWarning("tensors", Setup->Tensors->GetDataTypeAsString());
    return false;
  }

  if (Setup->Eigenvectors->GetNumberOfComponents() != 2) {
    writeComponentWarning("line", "vectors");
    return false;
  }

  if (Setup->Tensors->GetNumberOfComponents() != 4) {
    writeComponentWarning("tensor", "tensors");
    return false;
  }

  return true;
}

//----------------------------------------------------------------------------

void vtkTensorFieldCriticalCells::BuildEdgeTable() {
  EdgeTableBuildFunctor edgeTableBuilder(Setup);

  if (!Dispatcher::Execute(Setup->Eigenvectors, edgeTableBuilder)) {
    edgeTableBuilder(Setup->Eigenvectors.GetPointer());
  }
}

//----------------------------------------------------------------------------

bool vtkTensorFieldCriticalCells::CheckLineField() const {
  vtkIdType p1{0};
  vtkIdType p2{0};

  void* edgeInfoPtr{nullptr};

  Setup->EdgeTable->InitTraversal();

  while (Setup->EdgeTable->GetNextEdge(p1, p2, edgeInfoPtr)) {
    auto edgeInfo = static_cast<EdgeInformation*>(edgeInfoPtr);
    if (edgeInfo->Value == EdgeValue::Perpendicular) {
      return false;
    }
  }

  return true;
}

//----------------------------------------------------------------------------

void vtkTensorFieldCriticalCells::PerturbateLineField() {
  LineFieldPerturbationFunctor lineFieldPerturbator(Setup);

  if (!Dispatcher::Execute(Setup->Eigenvectors, lineFieldPerturbator)) {
    lineFieldPerturbator(Setup->Eigenvectors.GetPointer());
  }
}

//----------------------------------------------------------------------------

void vtkTensorFieldCriticalCells::ClassifyEdges() {
  EdgeClassificationFunctor edgeClassificationFunctor(Setup);

  if (!Dispatcher::Execute(Setup->Tensors, edgeClassificationFunctor)) {
    edgeClassificationFunctor(Setup->Tensors.GetPointer());
  }
}

//----------------------------------------------------------------------------

void vtkTensorFieldCriticalCells::ClassifyCells() {
  const auto numberOfCells = Setup->InField->GetNumberOfCells();

  auto pointIDs = vtkSmartPointer<vtkIdList>::New();

  for (vtkIdType cellIndex{0}; cellIndex < numberOfCells; ++cellIndex) {
    pointIDs->Resize(0);
    Setup->InField->GetCellPoints(cellIndex, pointIDs);

    vtkIdType cellValue{1};

    std::array<vtkTensorFieldCriticalCells::EdgeInformation*, 3> edgeInfos{nullptr, nullptr, nullptr};

    for (vtkIdType i{0}; i < 3; ++i) {
      const auto p1 = pointIDs->GetId(i);
      const auto p2 = pointIDs->GetId((i + 1) % 3);

      void* edgeInfoPtr{nullptr};
      Setup->EdgeTable->IsEdge(p1, p2, edgeInfoPtr);
      if (edgeInfoPtr) {
        edgeInfos[i] = static_cast<vtkTensorFieldCriticalCells::EdgeInformation*>(edgeInfoPtr);

        cellValue *= static_cast<vtkIdType>(edgeInfos[i]->Value);
      }
    }

    if (cellValue < 0) {
      Setup->DegenerateCellsFlags->InsertNextValue(static_cast<int>(1));

      EdgeRotation direction{EdgeRotation::Uninitialized};

      for (auto edgeInfo : edgeInfos) {
        direction = edgeInfo->Rotation;
        if (direction != EdgeRotation::None) break;
      }

      if (direction == EdgeRotation::None)
        vtkErrorMacro(<< "Degenerate cell without rotation. Result inconsistent.");

      Setup->DegenerateCellsTypes->InsertNextValue(static_cast<int>(direction));
    } else {
      Setup->DegenerateCellsFlags->InsertNextValue(static_cast<int>(0));
      Setup->DegenerateCellsTypes->InsertNextValue(static_cast<int>(EdgeRotation::None));
    }
  }
}

bool vtkTensorFieldCriticalCells::SubdivideMesh() {
  MeshSubdivisionFunctor meshSubdivisionFunctor(Setup);

  if (!Dispatcher::Execute(Setup->Tensors, meshSubdivisionFunctor)) {
    meshSubdivisionFunctor(Setup->Tensors.GetPointer());
  }

  return meshSubdivisionFunctor.Success;
}
