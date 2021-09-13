#pragma once

#include <inviwo/featurelevelsetsgl/featurelevelsetsglmoduledefine.h>
#include <inviwo/featurelevelsetsgl/properties/traitproperty.h>
#include <inviwo/core/properties/stringproperty.h>

namespace inviwo {
class IVW_MODULE_FEATURELEVELSETSGL_API ImplicitFunctionTraitProperty : public TraitProperty {
public:
    ImplicitFunctionTraitProperty() = delete;
    ImplicitFunctionTraitProperty(const ImplicitFunctionTraitProperty& p)
        : TraitProperty(p)
        , shaderInjection_("shaderInjection", "Implicit function", "",
                           InvalidationLevel::InvalidOutput, PropertySemantics::ShaderEditor) {
        addProperties(shaderInjection_);
    }

    ImplicitFunctionTraitProperty(const std::string& identifier, const std::string& displayName)
        : TraitProperty(identifier, displayName)
        , shaderInjection_("shaderInjection", "Implicit function", "",
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

    std::string get() const;

private:
    StringProperty shaderInjection_;
};

}  // namespace inviwo
