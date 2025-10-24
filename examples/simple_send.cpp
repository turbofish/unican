#include "../can_bus.hpp"
#include <memory>
#include <iostream>

using namespace unican;

int main(int argc, char** argv)
{
    // Use factory pattern to create the appropriate bus type
    std::string bus_type = "socketcan";  // Can be "socketcan", "kvaser", or "null"
    std::string interface_name = "vcan0";
    
    // Parse command line arguments if provided
    if (argc > 1) {
        bus_type = argv[1];
    }
    if (argc > 2) {
        interface_name = argv[2];
    }
    
    try {
        // Create bus using factory
        auto bus = createBus(bus_type, interface_name);
        
        std::cout << "Created bus: " << bus->getDeviceInfo() << std::endl;
        
        // Configure and start
        bus->setBitrate(500000);
        bus->start();
        
        // Send a test frame
        uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
        CanFrame frame(0x123, std::span<const uint8_t>{data, 4});
        bus->send(frame);
        
        std::cout << "Frame sent successfully!" << std::endl;
        
        // Stop the bus
        bus->stop();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
