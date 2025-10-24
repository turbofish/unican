/**
 * @file can_bus.hpp
 * @brief CAN interface abstraction for hardware communication
 * @author Tomas Hellström <turbofish@fripost.org>
 * @copyright Copyright (c) 2025 Tomas Hellström <turbofish@fripost.org>
 * @version 0.1
 * @date 2025-10-21
 * 
 * @note This file is part of the unican project.
 * @note This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This file defines the abstract interface for CAN communication, providing
 * a hardware-agnostic way to interact with CAN buses. It includes support
 * for both CAN 2.0 and CAN FD protocols, filtering, and asynchronous communication.
 */

 #pragma once

 #include "can_frame.hpp"
#include <string>
#include <memory>
#include <optional>
#include <chrono>

namespace unican {

/**
 * @struct CanFilter
 * @brief CAN frame filter for hardware-level message filtering
 * 
 * This structure defines a CAN frame filter that can be applied at the hardware level
 * to reduce CPU load by filtering unwanted messages before they reach the application.
 * The filter uses a bitmask approach where only frames matching the filter criteria
 * are passed through to the application.
 * 
 * @example
 * ```cpp
 * // Filter for CAN ID 0x123 (exact match)
 * CanFilter filter{0x123, 0x7FF};  // 0x7FF = 11 bits all set
 * 
 * // Filter for CAN IDs 0x100-0x1FF (range filter)
 * CanFilter rangeFilter{0x100, 0x700};  // 0x700 = mask for upper 3 bits
 * 
 * // Filter for extended frames with ID 0x12345678
 * CanFilter extFilter{0x12345678, 0x1FFFFFFF};  // 29-bit mask
 * ```
 */
struct CanFilter {
    /// CAN frame ID to match (11-bit for standard, 29-bit for extended)
    uint32_t id;
    
    /// Bitmask defining which bits to compare (1 = compare, 0 = don't care)
    uint32_t mask;
};

/**
 * @class ICanBus
 * @brief Abstract interface for CAN hardware communication
 * 
 * This interface provides a hardware-agnostic abstraction for CAN communication,
 * allowing applications to work with different CAN hardware implementations without
 * code changes. It supports both CAN 2.0 and CAN FD protocols, hardware filtering,
 * and asynchronous communication patterns.
 * 
 * The interface is designed to be implemented by hardware-specific drivers (e.g.,
 * SocketCAN, USB-CAN adapters, embedded CAN controllers) while providing a
 * consistent API for application code.
 * 
 * @note This is an abstract base class - concrete implementations must be provided
 * @note All methods are thread-safe unless otherwise specified
 * @note The interface supports both blocking and non-blocking operations
 * 
 * @example
 * ```cpp
 * // Create a CAN interface (implementation-specific)
 * auto canInterface = std::make_unique<SocketCanInterface>("can0");
 * 
 * // Configure the interface
 * canInterface->setBitrate(500000);  // 500 kbps
 * canInterface->goOnBus();
 * 
 * // Send a frame
 * CanFrame frame(0x123, {0x01, 0x02, 0x03, 0x04});
 * canInterface->send(frame);
 * 
 * // Receive frames with timeout
 * auto received = canInterface->receive(std::chrono::milliseconds(100));
 * if (received) {
 *     std::cout << "Received: " << received->toString() << std::endl;
 * }
 * ```
 */
class ICanBus {
public:
    /**
     * @brief Virtual destructor
     * 
     * Ensures proper cleanup of derived classes and their resources.
     * Automatically calls goOffBus() if the interface is still on the bus.
     */
    virtual ~ICanBus() = default;
    
    // Configuration
    /**
     * @brief Set the CAN bus bitrate
     * @param bitrate The desired bitrate in bits per second
     * 
     * Configures the CAN controller to operate at the specified bitrate.
     * Common bitrates include:
     * - 125000 bps (125 kbps)
     * - 250000 bps (250 kbps) 
     * - 500000 bps (500 kbps)
     * - 1000000 bps (1 Mbps)
     * 
     * @note This method must be called before goOnBus()
     * @note The actual bitrate may be limited by hardware capabilities
     * 
     * @throws std::invalid_argument if bitrate is not supported by hardware
     * @throws std::runtime_error if interface is already on bus
     */
    virtual void setBitrate(uint32_t bitrate) = 0;
    
    /**
     * @brief Bring the CAN interface online
     * 
     * Activates the CAN interface and starts listening for messages.
     * After calling this method, the interface can send and receive CAN frames.
     * 
     * @note Must be called after setBitrate()
     * @note Multiple calls are safe (idempotent)
     * 
     * @throws std::runtime_error if interface cannot be brought online
     * @throws std::runtime_error if bitrate has not been set
     */
    virtual void start() = 0;
    
    /**
     * @brief Take the CAN interface offline
     * 
     * Deactivates the CAN interface and stops all communication.
     * Any pending operations are cancelled and the interface becomes inactive.
     * 
     * @note Multiple calls are safe (idempotent)
     * @note Automatically called by destructor if still on bus
     */
    virtual void stop() = 0;
    
    // Communication
    /**
     * @brief Send a CAN frame
     * @param frame The CAN frame to send
     * 
     * Transmits the specified CAN frame on the bus. The frame is queued for
     * transmission and sent as soon as the bus is available.
     * 
     * @note Interface must be on bus (goOnBus() called)
     * @note Frame is copied internally, so original can be modified after call
     * @note Transmission is asynchronous - method returns immediately
     * 
     * @throws std::runtime_error if interface is not on bus
     * @throws std::runtime_error if transmission queue is full
     * @throws std::invalid_argument if frame is malformed
     */
    virtual void send(const CanFrame& frame) = 0;
    
    /**
     * @brief Receive a CAN frame
     * @param timeout Maximum time to wait for a frame (default: wait indefinitely)
     * @return Optional containing the received frame, or empty if timeout
     * 
     * Receives a CAN frame from the bus. If no frame is available immediately,
     * the method will wait for the specified timeout period.
     * 
     * @note Interface must be on bus (goOnBus() called)
     * @note Only frames matching active filters are returned
     * @note Timeout of -1 means wait indefinitely
     * 
     * @example
     * ```cpp
     * // Wait up to 100ms for a frame
     * auto frame = interface->receive(std::chrono::milliseconds(100));
     * if (frame) {
     *     std::cout << "Received: " << frame->toString() << std::endl;
     * }
     * 
     * // Wait indefinitely for a frame
     * auto frame = interface->receive();
     * ```
     * 
     * @throws std::runtime_error if interface is not on bus
     */
    virtual std::optional<CanFrame> receive(
        std::chrono::milliseconds timeout = std::chrono::milliseconds(-1)) = 0;
    
    // Filtering
    /**
     * @brief Set a hardware filter for incoming frames
     * @param filter The filter configuration to apply
     * 
     * Configures hardware-level filtering to reduce CPU load by only passing
     * frames that match the specified criteria to the application.
     * 
     * @note Multiple filters can be set (hardware dependent)
     * @note Filters are applied at the hardware level for efficiency
     * @note Can be called before or after going on bus
     * 
     * @example
     * ```cpp
     * // Filter for exact CAN ID 0x123
     * CanFilter exactFilter{0x123, 0x7FF};
     * interface->setFilter(exactFilter);
     * 
     * // Filter for range 0x100-0x1FF
     * CanFilter rangeFilter{0x100, 0x700};
     * interface->setFilter(rangeFilter);
     * ```
     * 
     * @throws std::runtime_error if filter cannot be set (hardware limitation)
     */
    virtual void setFilter(const CanFilter& filter) = 0;
    
    /**
     * @brief Clear all hardware filters
     * 
     * Removes all active filters, allowing all frames to be received.
     * This is useful for debugging or when you need to receive all traffic.
     * 
     * @note Safe to call even when no filters are active
     * @note Can be called before or after going on bus
     */
    virtual void clearFilters() = 0;
    
    // Status
    /**
     * @brief Check if the interface is currently on the bus
     * @return true if interface is active and can communicate, false otherwise
     * 
     * Indicates whether the CAN interface is currently active and able to
     * send/receive frames. This is useful for checking interface state
     * before attempting communication.
     * 
     * @note Thread-safe
     * @note Returns false if interface has never been brought online
     * @note Returns false if interface has been taken offline
     */
    virtual bool isStarted() const = 0;
    
    /**
     * @brief Get device information string
     * @return Human-readable string describing the hardware interface
     * 
     * Returns a descriptive string containing information about the
     * underlying CAN hardware, such as device name, driver version,
     * or hardware capabilities.
     * 
     * @note Format is implementation-specific
     * @note Useful for debugging and logging
     * 
     * @example
     * ```cpp
     * std::cout << "Using interface: " << interface->getDeviceInfo() << std::endl;
     * // Output: "SocketCAN interface: can0 (500 kbps)"
     * ```
     */
    virtual std::string getDeviceInfo() const = 0;
    
    // Prevent copying, allow moving
    /**
     * @brief Deleted copy constructor
     * 
     * CAN interfaces are non-copyable to prevent resource conflicts.
     * Use move semantics or smart pointers for interface management.
     */
    ICanBus(const ICanBus&) = delete;
    
    /**
     * @brief Deleted copy assignment operator
     * 
     * CAN interfaces are non-copyable to prevent resource conflicts.
     * Use move semantics or smart pointers for interface management.
     */
    ICanBus& operator=(const ICanBus&) = delete;
    
    /**
     * @brief Default move constructor
     * 
     * Allows efficient transfer of interface ownership without copying.
     * The moved-from object becomes invalid and should not be used.
     */
    ICanBus(ICanBus&&) = default;
    
    /**
     * @brief Default move assignment operator
     * 
     * Allows efficient transfer of interface ownership without copying.
     * The moved-from object becomes invalid and should not be used.
     */
    ICanBus& operator=(ICanBus&&) = default;
    
protected:
    /**
     * @brief Protected default constructor
     * 
     * Prevents direct instantiation of the abstract base class.
     * Only derived classes can be instantiated.
     */
    ICanBus() = default;
};

/**
 * @brief Factory function to create CAN bus instances
 * @param bus_type The type of CAN bus to create ("socketcan", "kvaser", "null")
 * @param interface_name The interface name (e.g., "can0", "vcan0")
 * @return Unique pointer to the created CAN bus interface
 * 
 * This factory function provides a unified way to create different types of CAN bus
 * interfaces without needing to know the specific implementation details.
 * 
 * @example
 * ```cpp
 * // Create a SocketCAN interface
 * auto bus = createBus("socketcan", "can0");
 * 
 * // Create a Kvaser interface
 * auto bus = createBus("kvaser", "can0");
 * 
 * // Create a null interface for testing
 * auto bus = createBus("null", "");
 * ```
 * 
 * @throws std::invalid_argument if bus_type is not supported
 * @throws std::runtime_error if interface creation fails
 */
std::unique_ptr<ICanBus> createBus(const std::string& bus_type, const std::string& interface_name);

} // namespace unican
