#pragma once

#include <inviwo/core/ports/inport.h>
#include <inviwo/core/ports/outport.h>

namespace inviwo {

template <typename T>
bool checkPort(T& port) {
    return t.hasData() && t.getData();
}

template <typename... Ts>
bool checkPorts(Ts&... ports) {
    return true && (checkPort(ports), ...);
}
}  // namespace inviwo