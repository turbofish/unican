/**
 * @file interface_socketcan.hpp
 * @brief SocketCAN backend header for Linux CAN interfaces
 * @author Tomas Hellström <turbofish@fripost.org>
 * @copyright Copyright (c) 2025 Tomas Hellström <turbofish@fripost.org>
 * 
 * @note This file is part of the unican project.
 * @note This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This header defines the SocketCanBus class which provides SocketCAN support
 * for Linux systems, allowing communication with CAN devices through the
 * standard Linux SocketCAN interface.
 */

#pragma once

#include "can_bus.hpp"
#include "can_exception.hpp"
#include <string>
#include <array>

namespace unican {

/**
 * @class SocketCanBus
 * @brief SocketCAN implementation of the CAN bus interface
 * 
 * This class provides a complete implementation of the ICanBus interface
 * using the Linux SocketCAN subsystem. It supports both CAN 2.0 and
 * CAN FD protocols through the standard Linux CAN interface.
 * 
 * @note Requires Linux with SocketCAN support
 * @note Thread-safe for concurrent access
 * @note Supports both blocking and non-blocking operations
 * 
 * @example
 * ```cpp
 * // Create SocketCAN interface
 * auto canBus = std::make_unique<SocketCanBus>("can0");
 * 
 * // Configure and start
 * canBus->setBitrate(500000);
 * canBus->start();
 * 
 * // Send a frame
 * CanFrame frame(0x123, {0x01, 0x02, 0x03, 0x04});
 * canBus->send(frame);
 * 
 * // Receive frames
 * auto received = canBus->receive(std::chrono::milliseconds(100));
 * ```
 */
class SocketCanBus : public ICanBus {
public:
    /**
     * @brief Construct a SocketCAN interface
     * @param interface_name The CAN interface name (e.g., "can0", "vcan0")
     * 
     * @throws CanBusException if interface cannot be opened
     */
    explicit SocketCanBus(const std::string& interface_name);
    
    /**
     * @brief Destructor
     * 
     * Automatically stops the interface and closes the socket.
     */
    ~SocketCanBus() override;
    
    /**
     * @brief Set the CAN bus bitrate
     * @param bitrate The desired bitrate in bits per second
     * 
     * @note This is a placeholder - actual bitrate setting requires
     *       system-level configuration (e.g., using ip link commands)
     */
    void setBitrate(uint32_t bitrate) override;
    
    /**
     * @brief Start the CAN interface
     * 
     * Opens the SocketCAN socket and brings the interface online.
     * 
     * @throws CanBusException if socket cannot be opened
     * @throws CanBusException if interface is not available
     */
    void start() override;
    
    /**
     * @brief Stop the CAN interface
     * 
     * Closes the SocketCAN socket and takes the interface offline.
     */
    void stop() override;
    
    /**
     * @brief Send a CAN frame
     * @param frame The CAN frame to send
     * 
     * @throws CanBusException if interface is not started
     * @throws CanBusException if transmission fails
     */
    void send(const CanFrame& frame) override;
    
    /**
     * @brief Receive a CAN frame
     * @param timeout Maximum time to wait for a frame (default: wait indefinitely)
     * @return Optional containing the received frame, or empty if timeout
     * 
     * @throws CanBusException if interface is not started
     */
    std::optional<CanFrame> receive(std::chrono::milliseconds timeout = std::chrono::milliseconds(-1)) override;
    
    /**
     * @brief Set a hardware filter
     * @param filter The filter to apply
     * 
     * @note SocketCAN filtering is implemented at the kernel level
     */
    void setFilter(const CanFilter& filter) override;
    
    /**
     * @brief Clear all hardware filters
     * 
     * Removes all filters, allowing all frames to be received.
     */
    void clearFilters() override;
    
    /**
     * @brief Check if interface is started
     * @return true if interface is active, false otherwise
     */
    bool isStarted() const override;
    
    /**
     * @brief Get device information
     * @return String describing the SocketCAN interface
     */
    std::string getDeviceInfo() const override;
    
private:
    std::string interface_name_;  ///< CAN interface name (e.g., "can0")
    int socket_fd_;              ///< Socket file descriptor
    bool is_started_;            ///< Interface start state
    uint32_t bitrate_;           ///< Configured bitrate
};

} // namespace unican
