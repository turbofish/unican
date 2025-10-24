#include "can_bus.hpp"
#include "bus_socketcan.hpp"
#include "bus_kvaser.hpp"
#include "bus_null.hpp"
#include "can_exception.hpp"
#include <stdexcept>

namespace unican {

std::unique_ptr<ICanBus> createBus(const std::string& bus_type, const std::string& interface_name) {
    if (bus_type == "socketcan") {
        return std::make_unique<SocketCanBus>(interface_name);
    }
    else if (bus_type == "kvaser") {
        return std::make_unique<KvaserCanBus>(interface_name);
    }
    else if (bus_type == "null") {
        return std::make_unique<NullCanBus>(interface_name);
    }
    else {
        throw std::invalid_argument("Unsupported bus type: " + bus_type);
    }
}

} // namespace unican
