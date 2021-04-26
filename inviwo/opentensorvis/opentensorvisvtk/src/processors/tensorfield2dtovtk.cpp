/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2019-2020 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <inviwo/opentensorvisvtk/processors/tensorfield2dtovtk.h>

#include <warn/push>
#include <warn/ignore/all>
#include <vtkStructuredGrid.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkDoubleArray.h>
#include <vtkAOSDataArrayTemplate.h>
#include <warn/pop>

#include <inviwo/opentensorvisbase/opentensorvisbasemodule.h>
#include <inviwo/vtk/vtkmodule.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo TensorField2DToVTK::processorInfo_{
    "org.inviwo.TensorField2DToVTK",             // Class identifier
    "Tensor Field 2D To VTK",                    // Display name
    "VTK",                                       // Category
    CodeState::Experimental,                     // Code state
    tag::OpenTensorVis | Tag::CPU | Tag("VTK"),  // Tags
};
const ProcessorInfo TensorField2DToVTK::getProcessorInfo() const { return processorInfo_; }

TensorField2DToVTK::TensorField2DToVTK()
    : Processor()
    , tensorFieldInport_("tensorFieldInport")
    , vtkDataSetOutport_("vtkDataSetOutport") {

    addPort(tensorFieldInport_);
    addPort(vtkDataSetOutport_);
}

void TensorField2DToVTK::process() {
    const auto tensorField = tensorFieldInport_.getData();
    const auto dimensions = ivec3{tensorField->getDimensionsAs<int>(), 1};
    const auto &tensors = *tensorField->tensors();

    auto structuredGrid = vtkSmartPointer<vtkStructuredGrid>::New();
    auto points = vtkSmartPointer<vtkPoints>::New();

    for (int y{0}; y < dimensions[1]; ++y) {
        for (int x{0}; x < dimensions[0]; ++x) {
            points->InsertNextPoint(x, y, 0);
        }
    }

    structuredGrid->SetDimensions(glm::value_ptr(dimensions));
    structuredGrid->SetPoints(points);

    auto pointData = structuredGrid->GetPointData();

    auto tensor2DArray =
        vtkSmartPointer<vtkAOSDataArrayTemplate<typename TensorField2D::value_type>>::New();
    tensor2DArray->SetNumberOfComponents(4);
    tensor2DArray->SetNumberOfTuples(tensors.size());

    tensor2DArray->SetComponentName(0, "xx");
    tensor2DArray->SetComponentName(1, "xy");
    tensor2DArray->SetComponentName(2, "yx");
    tensor2DArray->SetComponentName(3, "yy");
    tensor2DArray->SetName("Tensors 2D");

    std::memcpy(tensor2DArray->GetPointer(0), tensors.data(),
                sizeof(typename TensorField2D::matN) * tensors.size());

    pointData->AddArray(tensor2DArray);

    if (auto eigenvectorColumn = tensorField->getMetaData<attributes::MajorEigenVector2D>()) {
        auto typedColumn =
            std::dynamic_pointer_cast<const TemplateColumn<typename TensorField2D::vecN>>(
                *eigenvectorColumn);

        const auto dataPointer =
            typedColumn->getTypedBuffer()->getRAMRepresentation()->getDataContainer().data();

        auto eigenvector2DArray =
            vtkSmartPointer<vtkAOSDataArrayTemplate<typename TensorField2D::value_type>>::New();
        eigenvector2DArray->SetNumberOfComponents(2);
        eigenvector2DArray->SetNumberOfTuples(tensors.size());
        eigenvector2DArray->SetComponentName(0, "x");
        eigenvector2DArray->SetComponentName(1, "y");
        eigenvector2DArray->SetName("Eigenvectors 2D");

        std::memcpy(eigenvector2DArray->GetPointer(0), dataPointer,
                    sizeof(typename TensorField2D::vecN) * tensors.size());

        pointData->AddArray(eigenvector2DArray);

        auto eigenvector3DArray =
            vtkSmartPointer<vtkAOSDataArrayTemplate<typename TensorField2D::value_type>>::New();
        eigenvector3DArray->SetNumberOfComponents(3);
        eigenvector3DArray->SetNumberOfTuples(tensors.size());
        eigenvector3DArray->SetComponentName(0, "x");
        eigenvector3DArray->SetComponentName(1, "y");
        eigenvector3DArray->SetComponentName(2, "z");
        eigenvector3DArray->SetName("Eigenvectors 3D");

        std::memset(eigenvector3DArray->GetPointer(0), 0, sizeof(typename TensorField2D::value_type) * tensors.size());

        for (size_t i{0}; i < tensors.size(); i+=2) {
            std::memcpy(eigenvector3DArray->GetPointer(i + i / 2), dataPointer + i,
                        sizeof(typename TensorField2D::vecN) * 2);
        }
    }

    vtkDataSetOutport_.setData(std::make_shared<VTKDataSet>(structuredGrid));
}

}  // namespace inviwo
