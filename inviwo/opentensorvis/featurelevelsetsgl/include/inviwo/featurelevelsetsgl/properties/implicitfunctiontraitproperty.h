#pragma once

#include <inviwo/featurelevelsetsgl/featurelevelsetsglmoduledefine.h>
#include <inviwo/featurelevelsetsgl/properties/traitproperty.h>
#include <inviwo/core/properties/stringproperty.h>
#include <inviwo/core/properties/buttonproperty.h>

namespace inviwo {
class IVW_MODULE_FEATURELEVELSETSGL_API ImplicitFunctionTraitProperty : public TraitProperty {
public:
    ImplicitFunctionTraitProperty() = delete;
    ImplicitFunctionTraitProperty(const ImplicitFunctionTraitProperty& p)
        : ImplicitFunctionTraitProperty(p.getIdentifier(), p.getDisplayName()) {}

    ImplicitFunctionTraitProperty(const std::string& identifier, const std::string& displayName)
        : TraitProperty(identifier, displayName)
        , shaderInjection_("shaderInjection", "Implicit function", "return 0.0;",
                           InvalidationLevel::InvalidOutput, PropertySemantics::ShaderEditor) {
        addProperties(shaderInjection_);
    }

    virtual ImplicitFunctionTraitProperty* clone() const final {
        return new ImplicitFunctionTraitProperty(*this);
    }

    virtual std::string getClassIdentifier() const override;
    static const std::string classIdentifier;

    virtual ~ImplicitFunctionTraitProperty() = default;

    void addAttribute(const std::string& name, std::shared_ptr<const Volume> volume,
                      bool useVolumeDataMap) final;

    void inject(Shader& shader);

private:
    StringProperty shaderInjection_;
    std::string originalShader_;
};

}  // namespace inviwo
