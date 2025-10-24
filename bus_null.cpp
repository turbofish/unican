#include "bus_null.hpp"
#include "can_exception.hpp"
#include <vector>

namespace unican {

NullCanBus::NullCanBus(const std::string& channel)
    : channel_(channel)
    , is_started_(false)
    , bitrate_(0)
    , tx_count_(0)
{
}
    
void NullCanBus::setBitrate(uint32_t bitrate) {
    bitrate_ = bitrate;
}
    
void NullCanBus::start() {
    is_started_ = true;
}
    
void NullCanBus::stop() {
    is_started_ = false;
}
    
void NullCanBus::send(const CanFrame& frame) {
    if (!is_started_) {
        throw CanBusException("Bus not started");
    }
    
    tx_count_++;
}
    
std::optional<CanFrame> NullCanBus::receive(std::chrono::milliseconds timeout) {
    if (!is_started_) {
        throw CanBusException("Bus not started");
    }
    
    return std::nullopt;
}
    
void NullCanBus::setFilter(const CanFilter& filter) {
    filters_.push_back(filter);
}
    
void NullCanBus::clearFilters() {
    filters_.clear();
}
    
bool NullCanBus::isStarted() const {
    return is_started_;
}
    
std::string NullCanBus::getDeviceInfo() const {
    return "Null CAN Bus (simulation) on " + channel_;
}

} // namespace unican