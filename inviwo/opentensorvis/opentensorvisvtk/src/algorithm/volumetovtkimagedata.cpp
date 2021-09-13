#include <inviwo/opentensorvisvtk/algorithm/volumetovtkimagedata.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>
#include <inviwo/core/util/formatdispatching.h>

namespace inviwo {
vtkSmartPointer<vtkImageData> VolumeToVTKImageData::convert(
    std::shared_ptr<const Volume> volume) const {
    const auto volumeRAM = volume->getRepresentation<VolumeRAM>();
    const auto dimensions = ivec3(volume->getDimensions());

    const auto basis = volume->getBasis();

    auto getExtents = [basis]() {
        return dvec3{glm::length(basis[0]), glm::length(basis[1]), glm::length(basis[2])};
    };

    auto getBounds = [dimensions]() { return dvec3(dimensions - ivec3{1}); };

    auto getSpacing = [getExtents, getBounds]() { return getExtents() / getBounds(); };

    const auto origin = volume->getOffset();

    const auto spacing = getSpacing();

    auto imageData = volumeRAM->dispatch<vtkSmartPointer<vtkImageData>, dispatching::filter::All>(
        [this, dimensions, spacing, origin](auto vrprecision) {
            auto imageData = vtkSmartPointer<vtkImageData>::New();
            imageData->SetDimensions(glm::value_ptr(dimensions));
            imageData->SetSpacing(glm::value_ptr(spacing));
            imageData->SetOrigin(origin[0], origin[1], origin[2]);

            /*
             * This gives the template argument type for VolumeRamPrecision, e.g. vec3
             */
            using Type = util::PrecisionValueType<decltype(vrprecision)>;
            /*
             * This extracts the template argument type for glm types, e.g. float for vec3
             */
            using Precision = typename util::value_type<Type>::type;

            constexpr auto numberOfComponents = util::extent<Type>::value;

            if constexpr (std::is_same_v<Precision, char>) {
                imageData->AllocateScalars(VTK_CHAR, numberOfComponents);
            } else if constexpr (std::is_same_v<Precision, signed char>) {
                imageData->AllocateScalars(VTK_SIGNED_CHAR, numberOfComponents);
            } else if constexpr (std::is_same_v<Precision, unsigned char>) {
                imageData->AllocateScalars(VTK_UNSIGNED_CHAR, numberOfComponents);
            } else if constexpr (std::is_same_v<Precision, short>) {
                imageData->AllocateScalars(VTK_SHORT, numberOfComponents);
            } else if constexpr (std::is_same_v<Precision, unsigned short>) {
                imageData->AllocateScalars(VTK_UNSIGNED_SHORT, numberOfComponents);
            } else if constexpr (std::is_same_v<Precision, int>) {
                imageData->AllocateScalars(VTK_INT, numberOfComponents);
            } else if constexpr (std::is_same_v<Precision, unsigned int>) {
                imageData->AllocateScalars(VTK_UNSIGNED_INT, numberOfComponents);
            } else if constexpr (std::is_same_v<Precision, long>) {
                imageData->AllocateScalars(VTK_LONG, numberOfComponents);
            } else if constexpr (std::is_same_v<Precision, unsigned long>) {
                imageData->AllocateScalars(VTK_UNSIGNED_LONG, numberOfComponents);
            } else if constexpr (std::is_same_v<Precision, float>) {
                imageData->AllocateScalars(VTK_FLOAT, numberOfComponents);
            } else if constexpr (std::is_same_v<Precision, double>) {
                imageData->AllocateScalars(VTK_DOUBLE, numberOfComponents);
            } else if constexpr (std::is_same_v<Precision, long long>) {
                imageData->AllocateScalars(VTK_LONG_LONG, numberOfComponents);
            } else if constexpr (std::is_same_v<Precision, unsigned long long>) {
                imageData->AllocateScalars(VTK_UNSIGNED_LONG_LONG, numberOfComponents);
            } else {
                LogWarn("Data type not supported, returning empty vtkImageData object.");
                return imageData;
            }

            const auto* data = static_cast<const Precision*>(vrprecision->getData());

            std::memcpy(imageData->GetScalarPointer(), data,
                        glm::compMul(dimensions) * numberOfComponents * sizeof(Precision));

            return imageData;
        });

    return imageData;
}
}  // namespace inviwo
