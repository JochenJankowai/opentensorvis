#include <inviwo/featurelevelsetsgl/properties/implicitfunctiontraitproperty.h>

namespace inviwo {
const std::string ImplicitFunctionTraitProperty::classIdentifier =
    "org.inviwo.ImplicitFunctionTraitProperty";
std::string ImplicitFunctionTraitProperty::getClassIdentifier() const { return classIdentifier; }

void ImplicitFunctionTraitProperty::addAttribute(const std::string&, std::shared_ptr<const Volume>,
                                                 bool) {}

std::string ImplicitFunctionTraitProperty::get() const { return shaderInjection_.get(); }
}  // namespace inviwo
