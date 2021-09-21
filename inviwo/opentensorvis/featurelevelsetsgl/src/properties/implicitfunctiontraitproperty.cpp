#include <inviwo/featurelevelsetsgl/properties/implicitfunctiontraitproperty.h>
#include <inviwo/core/util/stringconversion.h>

namespace inviwo {
const std::string ImplicitFunctionTraitProperty::classIdentifier =
    "org.inviwo.ImplicitFunctionTraitProperty";
std::string ImplicitFunctionTraitProperty::getClassIdentifier() const { return classIdentifier; }

void ImplicitFunctionTraitProperty::addAttribute(const std::string&, std::shared_ptr<const Volume>,
                                                 bool) {}

void ImplicitFunctionTraitProperty::inject(Shader& shader) {
    const auto identifier = getIdentifier();
    const auto itIdentifier = std::find_if(identifier.rbegin(), identifier.rend(),
                                           [](char c) { return !std::isdigit(c); });
    std::string number(itIdentifier.base(), identifier.end());
    if (number.empty()) number = "1";

    if (originalShader_.empty()) {
        originalShader_ = shader.getShaderObject(ShaderType::Compute)->getResource()->source();
    }

    auto content = originalShader_;

    replaceInString(content, "// #define ENABLE_IMPLICIT_FUNCTION_" + number,
                    "#define ENABLE_IMPLICIT_FUNCTION_" + number);

    replaceInString(content, "// #IMPLICIT_FUNCTION_" + number, shaderInjection_.get());

    auto shaderResource = std::make_shared<StringShaderResource>("featurelevelsets.comp", content);

    Shader newShader{{{
                         ShaderType::Compute,
                         shaderResource,
                     }},
                     Shader::Build::No};
    newShader.getShaderObject(ShaderType::Compute)
        ->setShaderDefines(shader.getShaderObject(ShaderType::Compute)->getShaderDefines());

    shader = newShader;

    shader.build();
}

}  // namespace inviwo
