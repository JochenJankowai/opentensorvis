#pragma once

#include <inviwo/opentensorvisvtk/opentensorvisvtkmoduledefine.h>
#include <inviwo/core/datastructures/volume/volume.h>

#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>

namespace inviwo {
namespace dispatching {
namespace filter {

/**
 *	Matches all types except half float because it is not supported by VTK.
 */
template <typename Format>
struct AllButHalvsies
    : std::integral_constant<bool,
                             !(Format::numtype == NumericType::Float && Format::compsize == 2)> {};
}  // namespace filter
}  // namespace dispatching

class IVW_MODULE_OPENTENSORVISVTK_API VolumeToUnstructuredGrid {
public:
    VolumeToUnstructuredGrid() = default;

    virtual ~VolumeToUnstructuredGrid() = default;

    vtkSmartPointer<vtkUnstructuredGrid> convert(std::shared_ptr<const Volume> volume,
                                                 const std::string& name = "Volume data") const;
};
}  // namespace inviwo