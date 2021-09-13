/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2021 Inviwo Foundation
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

#include <inviwo/opentensorvisttk/processors/opentensorvisttktestprocessor.h>
#include <modules/opengl/volume/volumegl.h>

#include <warn/push>
#include <warn/ignore/all>
#include <vtkSmartPointer.h>
#include <ttkFTMTree.h>
#include <ttkTriangulation.h>
#include <ttkTriangulationAlgorithm.h>
#include <ttkPersistenceCurve.h>
#include <ttkPersistenceDiagram.h>
#include <vtkThreshold.h>
#include <ttkTopologicalSimplification.h>
#include <vtkNew.h>
#include <vtkDataSet.h>
#include <vtkDataObject.h>
#include <warn/pop>

#include <inviwo/opentensorvisvtk/algorithm/volumetovtkimagedata.h>
#include <inviwo/vtk/datastructures/vtkdataset.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo OpenTensorVisTTKTestProcessor::processorInfo_{
    "org.inviwo.OpenTensorVisTTKTestProcessor",  // Class identifier
    "OpenTensorVis TTK Test Processor",          // Display name
    "OpenTensorVis",                             // Category
    CodeState::Experimental,                     // Code state
    Tags::CPU,                                   // Tags
};
const ProcessorInfo OpenTensorVisTTKTestProcessor::getProcessorInfo() const {
    return processorInfo_;
}

OpenTensorVisTTKTestProcessor::OpenTensorVisTTKTestProcessor()
    : Processor()
    , vtkDataSetInport_("vtkDataSetInport")
    , volumeOutport_("volumeOutport")
    , treeType_("treeType", "Tree type",
                {{"split", "Split", TreeType::Split},
                 {"join", "Join", TreeType::Join},
                 {"contour", "Contour", TreeType::Contour},
                 {"joinsplit", "Join/Split", TreeType::Join_Split}},
                1)
    , forceOffset_("forceOffset", "Force offset", false)
    , threshold_("threshold", "Threshold", 0.05, 0.01, 1000.0, 0.01)
    , trigger_("trigger", "Go") {
    addPorts(vtkDataSetInport_, volumeOutport_);
    addProperties(treeType_, forceOffset_, threshold_, trigger_);

    trigger_.onChange([this]() { compute(); });
}

void OpenTensorVisTTKTestProcessor::process() {}

void OpenTensorVisTTKTestProcessor::compute() {
    auto dataSet = vtkDataSetInport_.getData()->operator*();

    // 2. computing the persistence curve
    vtkNew<ttkPersistenceCurve> curve{};
    curve->SetInputDataObject(dataSet);
    curve->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS,
                                  vtkDataSetAttributes::SCALARS);

    // 3. computing the persitence diagram
    vtkNew<ttkPersistenceDiagram> diagram{};
    diagram->SetInputDataObject(dataSet);
    diagram->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS,
                                    vtkDataSetAttributes::SCALARS);

    // 4. selecting the most persistent pairs
    vtkNew<vtkThreshold> persistentPairs{};
    persistentPairs->SetInputConnection(diagram->GetOutputPort());
    persistentPairs->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_CELLS,
                                            "Persistence");
    persistentPairs->ThresholdBetween(threshold_.get(), 999999);

    // 5. selecting the critical point pairs
    vtkNew<vtkThreshold> criticalPairs{};
    criticalPairs->SetInputConnection(persistentPairs->GetOutputPort());
    criticalPairs->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_CELLS,
                                          "PairIdentifier");
    criticalPairs->ThresholdBetween(0, 999999);

    // 6. simplifying the input data to remove non-persistent pairs
    vtkNew<ttkTopologicalSimplification> topologicalSimplification{};
    topologicalSimplification->SetInputDataObject(dataSet);
    topologicalSimplification->SetInputArrayToProcess(
        0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, vtkDataSetAttributes::SCALARS);
    topologicalSimplification->SetInputConnection(1, criticalPairs->GetOutputPort());

    // 7. generate contour tree
    vtkNew<ttkFTMTree> ftmTree{};
    ftmTree->SetInputConnection(topologicalSimplification->GetOutputPort());
    ftmTree->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS,
                                    vtkDataSetAttributes::SCALARS);
    ftmTree->SetTreeType(static_cast<int>(treeType_.get()));
    ftmTree->SetForceInputOffsetScalarField(forceOffset_.get());
    ftmTree->Update();

    auto domain = vtkImageData::SafeDownCast(ftmTree->GetOutputDataObject(2));

    auto seg = domain->GetPointData()->GetArray("SegmentationId");
    LogInfo(fmt::format("Range: [{}, {}]", seg->GetRange()[0], seg->GetRange()[1]));

    volumeOutport_.setData(std::make_shared<VTKDataSet>(domain));
}

}  // namespace inviwo
