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

#include <iterator>
#include <string>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <vtkArrayDispatch.h>
#include <vtkEdgeTable.h>
#include <vtkIntArray.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkTriangle.h>
#include <vtkPointSet.h>
#include <vtkPointSetAlgorithm.h>
#include <vtkSmartPointer.h>
#include <vtkTensorFieldCriticalCellsFilterModule.h> // For export macro
//#include <vtkTensorFieldCriticalCellsModule.h> // This works for me
#include <vtkBitArray.h>

class VTKTENSORFIELDCRITICALCELLSFILTER_EXPORT vtkTensorFieldCriticalCells
//class VTKTENSORFIELDCRITICALCELLS_EXPORT vtkTensorFieldCriticalCells // This works for me
  : public vtkPointSetAlgorithm
{
public:
  enum class EdgeRotation : vtkIdType
  {
    Positive = -1,
    Negative = 1,
    None = 0,
    Uninitialized = -2
  };

  enum class EdgeValue : vtkIdType
  {
    Wide = -1,
    Perpendicular = 0,
    Narrow = 1,
    Uninitialized = -2
  };

  struct EdgeInformation
  {
    EdgeInformation()
      : Rotation(EdgeRotation::Uninitialized)
      , Value(EdgeValue::Uninitialized)
    {
    }

    EdgeInformation(EdgeRotation rotation, EdgeValue value)
      : Rotation(rotation)
      , Value(value)
    {
    }

    EdgeRotation Rotation;
    EdgeValue Value;
  };

  struct AlgorithmSetup
  {
    AlgorithmSetup()
      : InField(nullptr)
      , OutField(nullptr)
      , Eigenvectors(nullptr)
      , Tensors(nullptr)
      , DegenerateCellsFlags(nullptr)
      , DegenerateCellsTypes(nullptr)
      , EdgeTable(nullptr)
    {
    }

    AlgorithmSetup(vtkSmartPointer<vtkPointSet> inField, vtkSmartPointer<vtkPointSet> outField,
      vtkSmartPointer<vtkDataArray> eigenvectors, vtkSmartPointer<vtkDataArray> tensors,
      vtkSmartPointer<vtkBitArray> degenerateCellFlags,
      vtkSmartPointer<vtkIntArray> degenerateCellTypes, vtkSmartPointer<vtkEdgeTable> edgeTable)
      : InField(inField)
      , OutField(outField)
      , Eigenvectors(eigenvectors)
      , Tensors(tensors)
      , DegenerateCellsFlags(degenerateCellFlags)
      , DegenerateCellsTypes(degenerateCellTypes)
      , EdgeTable(edgeTable)
    {
    }

    vtkSmartPointer<vtkPointSet> InField;
    vtkSmartPointer<vtkPointSet> OutField;
    vtkSmartPointer<vtkDataArray> Eigenvectors;
    vtkSmartPointer<vtkDataArray> Tensors;
    vtkSmartPointer<vtkBitArray> DegenerateCellsFlags;
    vtkSmartPointer<vtkIntArray> DegenerateCellsTypes;
    vtkSmartPointer<vtkEdgeTable> EdgeTable;
    std::vector<std::shared_ptr<EdgeInformation> > EdgeInformationVector;
  };

  // ------------ Start New Code ---------- //
  struct Edge
  {
    Edge() : Edge(-1, -1)
    {
    }

    Edge(vtkIdType p1, vtkIdType p2)
      : isDivided(false)
      , subdivisionFactor(-1)
      , anisValue(0)
      , newEdgeVertexId(-1)
    {
      if(p1 < p2){
        this->p1 = p1;
        this->p2 = p2;
      } else {
        this->p1 = p2;
        this->p2 = p1;
      }
    }

    vtkIdType p1;
    vtkIdType p2;
    bool isDivided;
    double subdivisionFactor;
    double anisValue;
    vtkIdType newEdgeVertexId;
  };

  struct Triangle
  {
    Triangle() : Triangle(-1, -1, -1)
    {
    }

    Triangle(vtkIdType p1, vtkIdType p2, vtkIdType p3)
      : p1(p1)
      , p2(p2)
      , p3(p3)
      , isDivided(false)
      , baryCenterU(-1)
      , baryCenterV(-1)
      , anisValue(0)
      , newTriVertexId(-1)
    {
    }

    vtkIdType p1;
    vtkIdType p2;
    vtkIdType p3;
    std::vector<int> triEdgeIndices;
    bool isDivided;
    double baryCenterU;
    double baryCenterV;
    double anisValue;
    vtkIdType newTriVertexId;
  };
  // ------------ End New Code ---------- //

  using Dispatcher = vtkArrayDispatch::DispatchByValueType<vtkArrayDispatch::Reals>;

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
  
  // New methods
  void addOutputTriangle(vtkCellArray *trianglesArr, vtkTriangle *triangleVtk, vtkIdType p1, vtkIdType p2, vtkIdType p3);

  template<typename TensorArray>
  int SubdivideMesh(
    vtkPolyData* outputMesh, vtkPointSet* inField, TensorArray* tensors)
  {
    std::vector<Edge> edges;
    int edgeCount = 0;
    std::vector<Triangle> triangles;
    
    typedef std::pair<vtkIdType, vtkIdType> pair;
    std::map<pair, int> pairMap; 
    
    auto pointIDs = vtkSmartPointer<vtkIdList>::New();
    const auto numberOfCells = inField->GetNumberOfCells();

    for (vtkIdType cellIndex{ 0 }; cellIndex < numberOfCells; ++cellIndex)
    {
      pointIDs->Resize(0);
      inField->GetCellPoints(cellIndex, pointIDs);

      std::vector<int> triEdgeIndices;

      for (vtkIdType i{ 0 }; i < 3; ++i)
      {
        const auto p1 = pointIDs->GetId(i);
        const auto p2 = pointIDs->GetId((i + 1) % 3);

        auto it = pairMap.find(std::make_pair(p1, p2));
        int edgeIndex = -1;
        
        if (it == pairMap.end()) 
        {
          Edge edge(p1, p2);
          edges.push_back(edge);
          pairMap[std::make_pair(p1, p2)] = edgeCount;
          pairMap[std::make_pair(p2, p1)] = edgeCount;
          edgeIndex = edgeCount;
          ++edgeCount;
        } 
        else 
        {
          edgeIndex = it->second;
        }

        triEdgeIndices.push_back(edgeIndex);
      }

      const auto p1 = pointIDs->GetId(0);
      const auto p2 = pointIDs->GetId(1);
      const auto p3 = pointIDs->GetId(2);

      Triangle triangle(p1, p2, p3);
      triangle.triEdgeIndices = triEdgeIndices;
      triangles.push_back(triangle);
    }

    const auto origNumberOfPoints = inField->GetNumberOfPoints();
    vtkSmartPointer<vtkPoints> newPoints = vtkSmartPointer<vtkPoints>::New();
    
    vtkSmartPointer<vtkDoubleArray> anisotropyArray = vtkSmartPointer<vtkDoubleArray>::New();
    anisotropyArray->SetNumberOfComponents(1);
    anisotropyArray->SetName("Anisotropy");

    vtkSmartPointer<vtkDoubleArray> outputTensorsArray = vtkSmartPointer<vtkDoubleArray>::New();
    outputTensorsArray->SetNumberOfComponents(4);
    outputTensorsArray->SetName("Tensors");
    outputTensorsArray->DeepCopy(tensors);

    vtkSmartPointer<vtkUnsignedCharArray> pointTypeArray = vtkSmartPointer<vtkUnsignedCharArray>::New();
    pointTypeArray->SetNumberOfComponents(1);
    pointTypeArray->SetName("Point Type");

    using ValueType = vtk::GetAPIType<TensorArray>;
    vtkDataArrayAccessor<TensorArray> tensorsAccessor(tensors);

    for (vtkIdType pointIndex{ 0 }; pointIndex < origNumberOfPoints; ++pointIndex)
    {
      ValueType tensor[4];
      tensorsAccessor.Get(pointIndex, tensor);
      
      ValueType E = tensor[0];
      ValueType F = tensor[1];
      ValueType G = tensor[2];

      ValueType anisotropy = std::sqrt((E-G) * (E-G) + 4 * F * F);
      anisotropyArray->InsertNextTuple1(anisotropy);

      auto coords = inField->GetPoints()->GetPoint(pointIndex);
      newPoints->InsertNextPoint(coords[0], coords[1], coords[2]);
      pointTypeArray->InsertNextTuple1(0);
    }

    vtkIdType edgePoints{ 0 };
    for (auto &edge : edges)
    {
      auto p1 = edge.p1;
      auto p2 = edge.p2;

      ValueType tensor1[4];
      ValueType tensor2[4];

      tensorsAccessor.Get(p1, tensor1);
      tensorsAccessor.Get(p2, tensor2);

      ValueType E1 = tensor1[0];
      ValueType F1 = tensor1[1];
      ValueType G1 = tensor1[2];

      ValueType E2 = tensor2[0];
      ValueType F2 = tensor2[1];
      ValueType G2 = tensor2[2];

      ValueType A = (E2 - E1 - G2 + G1) * (E2 - E1 - G2 + G1)
          + 4 * (F2 - F1) * (F2 - F1);
      ValueType B = 2 * (E2 - E1 - G2 + G1) * (E1 - G1)
          + 8 * F1 * (F2 - F1);
      ValueType C = (E1 - G1) * (E1 - G1) + 4 * F1 * F1;
      
      if(A != 0) 
      {
        ValueType t = -B / (2 * A);
        edge.subdivisionFactor = t;
        if(t > 0 && t < 1) 
        {
          edge.isDivided = true;
          edge.anisValue = std::sqrt(A * t * t + B * t + C);
          anisotropyArray->InsertNextTuple1(edge.anisValue);

          ValueType edgetensor[4];
          edgetensor[0] = edgetensor[3] = (1-t) * E1 + t * E2;
          edgetensor[1] = (1-t) * F1 + t * F2;
          edgetensor[2] = (1-t) * G1 + t * G2;
          outputTensorsArray->InsertNextTuple(edgetensor);

          double coords1[3];
          double coords2[3];

          newPoints->GetPoint(p1, coords1);
          newPoints->GetPoint(p2, coords2);

          double coordsEdgePoint[3];
          for (size_t i{ 0 }; i < 3; ++i)
          {
            coordsEdgePoint[i] = (1-t) * coords1[i] + t * coords2[i];
          }

          newPoints->InsertNextPoint(coordsEdgePoint);
          pointTypeArray->InsertNextTuple1(1);
          edge.newEdgeVertexId = origNumberOfPoints + edgePoints;
          ++edgePoints;
        }
      } 
    }

    vtkIdType triPoints{ 0 };
    for(auto &tri : triangles) 
    {
      auto p1 = tri.p1;
      auto p2 = tri.p2;
      auto p3 = tri.p3;

      ValueType tensor1[4];
      ValueType tensor2[4];
      ValueType tensor3[4];

      tensorsAccessor.Get(p1, tensor1);
      tensorsAccessor.Get(p2, tensor2);
      tensorsAccessor.Get(p3, tensor3);

      ValueType E1 = tensor1[0];
      ValueType F1 = tensor1[1];
      ValueType G1 = tensor1[2];

      ValueType E2 = tensor2[0];
      ValueType F2 = tensor2[1];
      ValueType G2 = tensor2[2];

      ValueType E3 = tensor3[0];
      ValueType F3 = tensor3[1];
      ValueType G3 = tensor3[2];

      ValueType Ex = E1 - E3;
      ValueType Fx = F1 - F3;
      ValueType Gx = G1 - G3;

      ValueType Ey = E2 - E3;
      ValueType Fy = F2 - F3;
      ValueType Gy = G2 - G3;

      ValueType Ec = E3;
      ValueType Fc = F3;
      ValueType Gc = G3;

      ValueType A = (Ex - Gx) * (Ex - Gx) + 4 * Fx * Fx;
      ValueType B = 2 * (Ex - Gx) * (Ey - Gy) + 8 * Fx * Fy;
      ValueType C = (Ey - Gy) * (Ey - Gy) + 4 * Fy * Fy;
      ValueType D = 2 * (Ex - Gx) * (Ec - Gc) + 8 * Fx * Fc;
      ValueType E = 2 * (Ey - Gy) * (Ec - Gc) + 8 * Fy * Fc;
      
      ValueType H = 4 * A * C - B * B;

      if(H != 0) 
      {
        ValueType alpha = (-2 * C * D + B * E) / H;
        ValueType beta = (-2 * A * E + B * D) / H;
        ValueType gamma = 1 - alpha - beta;
        tri.baryCenterU = alpha;
        tri.baryCenterV = beta;
        tri.isDivided = (alpha > 0 && beta > 0 && gamma > 0);
        if (tri.isDivided)
        {
          tri.anisValue = 0;
          anisotropyArray->InsertNextTuple1(0);

          ValueType triTensor[4];
          triTensor[0] = triTensor[3] = alpha * E1 + beta * E2 + gamma * E3;
          triTensor[1] = alpha * F1 + beta * F2 + gamma * F3;
          triTensor[2] = alpha * G1 + beta * G2 + gamma * G3;
          outputTensorsArray->InsertNextTuple(triTensor);

          double coords1[3];
          double coords2[3];
          double coords3[3];

          newPoints->GetPoint(p1, coords1);
          newPoints->GetPoint(p2, coords2);
          newPoints->GetPoint(p3, coords3);

          double coordsTriPoint[3];
          for (size_t i{ 0 }; i < 3; ++i)
          {
            coordsTriPoint[i] = alpha * coords1[i] + beta * coords2[i] + gamma * coords3[i];
          }

          newPoints->InsertNextPoint(coordsTriPoint);
          pointTypeArray->InsertNextTuple1(2);
          tri.newTriVertexId = origNumberOfPoints + edgePoints + triPoints;
          ++triPoints;
        }
      } 
    }

    vtkSmartPointer<vtkCellArray> trianglesArr = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkTriangle> triangleVtk = vtkSmartPointer<vtkTriangle>::New();

    for(const auto& tri : triangles) 
    {
      auto p1 = tri.p1;
      auto p2 = tri.p2;
      auto p3 = tri.p3;

      auto e1Id = tri.triEdgeIndices[0];
      auto e2Id = tri.triEdgeIndices[1];
      auto e3Id = tri.triEdgeIndices[2];

      auto e1 = edges[e1Id];
      auto e2 = edges[e2Id];
      auto e3 = edges[e3Id];

      vtkIdType p12{0}, p23{0}, p13{0}, p123{0};
      if(tri.isDivided) 
      {
        p123 = tri.newTriVertexId;
        if(e1.isDivided) 
        {
          p12 = e1.newEdgeVertexId;
          addOutputTriangle(trianglesArr, triangleVtk, p123, p1, p12);
          addOutputTriangle(trianglesArr, triangleVtk, p123, p12, p2);
        } 
        else 
        {
          addOutputTriangle(trianglesArr, triangleVtk, p123, p1, p2);
        }
        if(e2.isDivided) 
        {
          p23 = e2.newEdgeVertexId;
          addOutputTriangle(trianglesArr, triangleVtk, p123, p2, p23);
          addOutputTriangle(trianglesArr, triangleVtk, p123, p23, p3);
        } 
        else 
        {
          addOutputTriangle(trianglesArr, triangleVtk, p123, p2, p3);
        }
        if(e3.isDivided) 
        {
          p13 = e3.newEdgeVertexId;
          addOutputTriangle(trianglesArr, triangleVtk, p123, p3, p13);
          addOutputTriangle(trianglesArr, triangleVtk, p123, p13, p1);
        } 
        else 
        {
          addOutputTriangle(trianglesArr, triangleVtk, p123, p3, p1);
        }
      } 
      else 
      {
        int numCrits = 0;
        if(e1.isDivided) 
        {
          p12 = e1.newEdgeVertexId;
          numCrits++;
        }
        if(e2.isDivided) 
        {
          p23 = e2.newEdgeVertexId;
          numCrits++;
        }
        if(e3.isDivided) 
        {
          p13 = e3.newEdgeVertexId;
          numCrits++;
        }
        switch(numCrits) 
        {
          case 0:
            addOutputTriangle(trianglesArr, triangleVtk, p1, p2, p3);
            break;
          case 1:
            if(e1.isDivided) 
            {
              addOutputTriangle(trianglesArr, triangleVtk, p12, p3, p1);
              addOutputTriangle(trianglesArr, triangleVtk, p12, p3, p2);
            } 
            else if(e2.isDivided) 
            {
              addOutputTriangle(trianglesArr, triangleVtk, p23, p1, p2);
              addOutputTriangle(trianglesArr, triangleVtk, p23, p1, p3);
            } 
            else if(e3.isDivided) 
            {
              addOutputTriangle(trianglesArr, triangleVtk, p13, p2, p3);
              addOutputTriangle(trianglesArr, triangleVtk, p13, p2, p1);
            }
            break;
          case 2:
            if(!e1.isDivided) 
            {
              addOutputTriangle(trianglesArr, triangleVtk, p23, p13, p3);
              if(e2.anisValue < e3.anisValue) 
              {
                addOutputTriangle(trianglesArr, triangleVtk, p23, p13, p1);
                addOutputTriangle(trianglesArr, triangleVtk, p23, p2, p1);
              } 
              else 
              {
                addOutputTriangle(trianglesArr, triangleVtk, p13, p23, p2);
                addOutputTriangle(trianglesArr, triangleVtk, p13, p1, p2);
              }
            } 
            else if(!e2.isDivided) 
            {
              addOutputTriangle(trianglesArr, triangleVtk, p12, p13, p1);
              if(e1.anisValue < e3.anisValue) 
              {
                addOutputTriangle(trianglesArr, triangleVtk, p12, p13, p3);
                addOutputTriangle(trianglesArr, triangleVtk, p12, p2, p3);
              } 
              else 
              {
                addOutputTriangle(trianglesArr, triangleVtk, p13, p12, p2);
                addOutputTriangle(trianglesArr, triangleVtk, p13, p3, p2);
              }
            } 
            else if(!e3.isDivided) 
            {
              addOutputTriangle(trianglesArr, triangleVtk, p12, p23, p2);
              if(e1.anisValue < e2.anisValue) 
              {
                addOutputTriangle(trianglesArr, triangleVtk, p12, p23, p3);
                addOutputTriangle(trianglesArr, triangleVtk, p12, p1, p3);
              } 
              else 
              {
                addOutputTriangle(trianglesArr, triangleVtk, p23, p12, p1);
                addOutputTriangle(trianglesArr, triangleVtk, p23, p3, p1);
              }
            }
            break;
          case 3:
            auto val12 = e1.anisValue;
            auto val23 = e2.anisValue;
            auto val13 = e3.anisValue;
            if(val12 < val23 && val12 < val13) 
            {
              addOutputTriangle(trianglesArr, triangleVtk, p12, p1, p13);
              addOutputTriangle(trianglesArr, triangleVtk, p12, p13, p3);
              addOutputTriangle(trianglesArr, triangleVtk, p12, p3, p23);
              addOutputTriangle(trianglesArr, triangleVtk, p12, p23, p2);
            } 
            else if(val23 < val12 && val23 < val13) 
            {
              addOutputTriangle(trianglesArr, triangleVtk, p23, p2, p12);
              addOutputTriangle(trianglesArr, triangleVtk, p23, p12, p1);
              addOutputTriangle(trianglesArr, triangleVtk, p23, p1, p13);
              addOutputTriangle(trianglesArr, triangleVtk, p23, p13, p3);
            } 
            else 
            {
              addOutputTriangle(trianglesArr, triangleVtk, p13, p3, p23);
              addOutputTriangle(trianglesArr, triangleVtk, p13, p23, p2);
              addOutputTriangle(trianglesArr, triangleVtk, p13, p2, p12);
              addOutputTriangle(trianglesArr, triangleVtk, p13, p12, p1);
            }
            break;
        }
      }
    }
    
    outputMesh->SetPoints(newPoints);
    outputMesh->SetPolys(trianglesArr);
    std::cout << "Number of cells after subdivision: " << std::to_string(outputMesh->GetNumberOfCells())
              << std::endl;

    auto outPointData = outputMesh->GetPointData();

    outPointData->AddArray(anisotropyArray);
    outPointData->AddArray(outputTensorsArray);
    outPointData->AddArray(pointTypeArray);
    outPointData->Update();

    return 1;
  }

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

#endif // vtkTensorFieldCriticalCells_h