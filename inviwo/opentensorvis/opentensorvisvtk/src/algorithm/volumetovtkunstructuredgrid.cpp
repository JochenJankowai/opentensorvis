#include <inviwo/opentensorvisvtk/algorithm/volumetovtkunstructuredgrid.h>

// VTK INCLUDES
#include <warn/push>
#include <warn/ignore/all>
#include <vtkUnstructuredGrid.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkAOSDataArrayTemplate.h>
#include <vtkPointData.h>
#include <vtkDelaunay3D.h>
#include <warn/pop>

#include <inviwo/core/datastructures/volume/volumeramprecision.h>
#include <inviwo/core/util/formatdispatching.h>

namespace inviwo {
vtkSmartPointer<vtkUnstructuredGrid> VolumeToVTKUnstructuredGrid::convert(
    std::shared_ptr<const Volume> volume, const std::string& name) const {
    const auto volumeRAM = volume->getRepresentation<VolumeRAM>();

    const auto dimensions = volume->getDimensions();
    const auto basis = volume->getBasis();
    
    auto getExtents = [basis]() {
        return dvec3{glm::length(basis[0]), glm::length(basis[1]), glm::length(basis[2])};
    };

    auto getBounds = [dimensions]() { return dvec3(dimensions - size3_t{1}); };

    auto getSpacing = [getExtents, getBounds]() { return getExtents() / getBounds(); };

    const auto spacing = getSpacing();

    auto unstructuredGrid = volumeRAM->dispatch<vtkSmartPointer<vtkUnstructuredGrid>,
                                                dispatching::filter::AllButHalvsies>(
        [dimensions, spacing, name](auto vrprecision) {
            /*
             * This gives the template argument type for VolumeRamPrecision, e.g. vec3
             */
            using Type = util::PrecisionValueType<decltype(vrprecision)>;
            /*
             * This extracts the template argument type for glm types, e.g. float for vec3
             */
            using Precision = typename util::value_type<Type>::type;

            constexpr auto numberOfComponents = util::extent<Type>::value;

            auto unstructuredGrid = vtkSmartPointer<vtkUnstructuredGrid>::New();
            auto points = vtkSmartPointer<vtkPoints>::New();
            
            /*
             * For now I am always assuming an axis-aligned orthogonal basis
             */
            for (size_t z{0}; z < dimensions.z; ++z) {
                for (size_t y{0}; y < dimensions.y; ++y) {
                    for (size_t x{0}; x < dimensions.x; ++x) {
                        points->InsertNextPoint(static_cast<double>(x) * spacing.x,
                                                static_cast<double>(y) * spacing.y,
                                                static_cast<double>(z) * spacing.z);
                    }
                }
            }

            unstructuredGrid->SetPoints(points);

            auto dataArray = vtkSmartPointer<vtkAOSDataArrayTemplate<Precision>>::New();
            dataArray->SetNumberOfComponents(numberOfComponents);
            dataArray->SetNumberOfTuples(glm::compMul(dimensions));

            StrBuffer buf;

            for (vtkIdType i{0}; i < numberOfComponents; ++i) {
                dataArray->SetComponentName(i, buf.replace("Channel {}", i).c_str());
            }

            dataArray->SetName(name.c_str());

            const auto* data = static_cast<const Precision*>(vrprecision->getData());

            std::memcpy(dataArray->GetPointer(0), data,
                        glm::compMul(dimensions) * numberOfComponents * sizeof(Precision));

            auto pointData = unstructuredGrid->GetPointData();
            pointData->AddArray(dataArray);

            return unstructuredGrid;
        });

    auto delaunay3DFilter = vtkSmartPointer<vtkDelaunay3D>::New();

    delaunay3DFilter->SetInputDataObject(0, unstructuredGrid);
    delaunay3DFilter->Update();
    auto triangulatedDomain = delaunay3DFilter->GetOutput();

    unstructuredGrid->CopyStructure(triangulatedDomain);

    return unstructuredGrid;
}
}  // namespace inviwo