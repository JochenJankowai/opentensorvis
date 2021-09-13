#pragma once

#include <inviwo/opentensorvisvtk/opentensorvisvtkmoduledefine.h>
#include <inviwo/core/datastructures/volume/volume.h>

#include <warn/push>
#include <warn/ignore/all>

#include <vtkSmartPointer.h>
#include <vtkImageData.h>

#include <warn/pop>

namespace inviwo {
class IVW_MODULE_OPENTENSORVISVTK_API VolumeToVTKImageData {
public:
    VolumeToVTKImageData() = default;
    ~VolumeToVTKImageData() = default;

    vtkSmartPointer<vtkImageData> convert(std::shared_ptr<const Volume> volume) const;
};
}  // namespace inviwo
