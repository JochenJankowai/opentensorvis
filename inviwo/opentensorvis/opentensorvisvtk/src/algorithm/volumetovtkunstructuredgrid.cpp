#include <inviwo/opentensorvisvtk/algorithm/volumetovtkunstructuredgrid.h>

// VTK INCLUDES
#include <warn/push>
#include <warn/ignore/all>
#include <vtkUnstructuredGrid.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkAOSDataArrayTemplate.h>
#include <vtkPointData.h>
#include <warn/pop>

namespace inviwo {
vtkSmartPointer<vtkUnstructuredGrid> VolumeToUnstructuredGrid::convert(
    std::shared_ptr<const Volume> volume, const std::string& name) const {
    const auto volumeRAM = volume->getRepresentation<VolumeRAM>();

    const auto dimensions = volume->getDimensions();
    const auto basis = volume->getBasis();
    const auto basisVectors =
        mat3{glm::normalize(basis[0]), glm::normalize(basis[1]), glm::normalize(basis[2])};

    auto getExtents = [basis]() {
        return vec3{glm::length(basis[0]), glm::length(basis[1]), glm::length(basis[2])};
    };

    auto getBounds = [dimensions]() { return vec3(dimensions - size3_t{1}); };

    auto getSpacing = [getExtents, getBounds]() { return getExtents() / getBounds(); };

    const auto spacing = getSpacing();

    auto unstructuredGrid = volumeRAM->dispatch<vtkSmartPointer<vtkUnstructuredGrid>,
                                                dispatching::filter::AllButHalvsies>(
        [dimensions, spacing, basisVectors, name](auto vrprecision) {
            using Type = typename util::PrecisionValueType<decltype(vrprecision)>::type;
            using Precision = typename util::value_type<Type>::type;

            constexpr auto numberOfComponents = util::extent<Type>::value;

            auto unstructuredGrid = vtkSmartPointer<vtkUnstructuredGrid>::New();
            auto points = vtkSmartPointer<vtkPoints>::New();

            const auto fBounds = dvec3(dimensions - size3_t{1});

            for (size_t z{0}; z < dimensions.z; ++z) {
                for (size_t y{0}; y < dimensions.y; ++y) {
                    for (size_t x{0}; x < dimensions.x; ++x) {
                        points->InsertNextPoint(static_cast<double>(x) / fBounds.x,
                                                static_cast<double>(y) / fBounds.y,
                                                static_cast<double>(z) / fBounds.z);
                    }
                }
            }

            unstructuredGrid->SetPoints(points);

            auto dataArray = vtkSmartPointer<vtkAOSDataArrayTemplate<Precision>>::New();
            dataArray->SetNumberOfComponents(numberOfComponents);
            dataArray->SetNumberOfTuples(glm::compMul(dimensions));

            for (vtkIdType i{0}; i < numberOfComponents; ++i) {
                dataArray->SetComponentName(i, StrBuffer{"Channel {}", i}.c_str());
            }

            dataArray->SetName(name.c_str());

            const auto* data = static_cast<const Precision*>(vrprecision->getData());

            std::memcpy(dataArray->GetPointer(0), data,
                        glm::compMul(dimensions) * numberOfComponents * sizeof(Precision));

            auto pointData = unstructuredGrid->GetPointData();
            pointData->AddArray(dataArray);

            return unstructuredGrid;
        });

    return unstructuredGrid;
}
}  // namespace inviwo