#include <inviwo/opentensorviscompute/algorithm/volumejacobiangl.h>

namespace inviwo {
inviwo::VolumeJacobianGL::VolumeJacobianGL()
    : shader_({{ShaderType::Compute, utilgl::findShaderResource("volumejacobian.comp")}},
              Shader::Build::Yes) {}

std::vector<std::shared_ptr<Volume>> inviwo::VolumeJacobianGL::compute(
    std::shared_ptr<const Volume> volume) {
    if (volume->getDataFormat()->getComponents() == 0 ||
        volume->getDataFormat()->getComponents() == 4) {
        LogErrorCustom("VolumeJacobianGL", "Input volume is not a 2D or 3D vector field. Returning empty set.")
        return std::vector<std::shared_ptr<Volume>>{};
    }

    auto outVolume = volume->getRepresentation<VolumeRAM>()
                    ->dispatch<std::shared_ptr<Volume>, dispatching::filter::Vecs>(
                        [](auto vrprecision) {
                            using ValueType = typename util::PrecisionValueType<decltype(vrprecision)>;
                            
                            using P = typename util::same_extent_t<ValueType, float>;

                            return std::make_shared<Volume>(
                                vrprecision->getDimensions(), DataFormat<P>::get(),
                                vrprecision->getSwizzleMask(), vrprecision->getInterpolation(),
                                vrprecision->getWrapping());
                        });

    outVolume->setModelMatrix(volume->getModelMatrix());
    outVolume->setWorldMatrix(volume->getWorldMatrix());
    outVolume->copyMetaDataFrom(*volume);


}
}  // namespace inviwo