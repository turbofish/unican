/**
 * @file interface_kvaser.hpp
 * @brief Kvaser CANlib backend header for Kvaser CAN interfaces
 * @author Tomas Hellström <turbofish@fripost.org>
 * @copyright Copyright (c) 2025 Tomas Hellström <turbofish@fripost.org>
 * 
 * @note This file is part of the unican project.
 * @note This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This header defines the KvaserCanBus class which provides Kvaser CANlib support
 * for Kvaser CAN interfaces, allowing communication with Kvaser hardware through
 * the Kvaser CANlib API.
 */

#pragma once

#include "can_bus.hpp"
#include "can_exception.hpp"
#include <string>
#include <array>
#include <vector>


namespace unican {

/**
 * @class KvaserCanBus
 * @brief Kvaser CANlib implementation of the CAN bus interface
 * 
 * This class provides a complete implementation of the ICanBus interface
 * using the Kvaser CANlib API. It supports both CAN 2.0 and CAN FD protocols
 * through Kvaser hardware interfaces.
 * 
 * @note Requires Kvaser CANlib library
 * @note Thread-safe for concurrent access
 * @note Supports both blocking and non-blocking operations
 * 
 * @example
 * ```cpp
 * // Create Kvaser interface
 * auto canBus = std::make_unique<KvaserCanBus>(0);  // Channel 0
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
class KvaserCanBus : public ICanBus {
public:
    /**
     * @brief Construct a Kvaser CAN interface
     * @param channel The Kvaser channel number (0, 1, 2, etc.)
     * 
     * @throws CanBusException if channel is invalid
     */
    explicit KvaserCanBus(int channel);
    
    /**
     * @brief Destructor
     * 
     * Automatically stops the interface and closes the channel.
     */
    ~KvaserCanBus() override;
    
    /**
     * @brief Set the CAN bus bitrate
     * @param bitrate The desired bitrate in bits per second
     * 
     * Configures the Kvaser interface to operate at the specified bitrate.
     * Common bitrates are supported through predefined constants.
     * 
     * @note Must be called before start()
     * @note The actual bitrate may be limited by hardware capabilities
     * 
     * @throws CanBusException if bitrate is not supported
     */
    void setBitrate(uint32_t bitrate) override;
    
    /**
     * @brief Start the CAN interface
     * 
     * Opens the Kvaser channel and brings the interface online.
     * 
     * @throws CanBusException if channel cannot be opened
     * @throws CanBusException if interface cannot be brought online
     */
    void start() override;
    
    /**
     * @brief Stop the CAN interface
     * 
     * Closes the Kvaser channel and takes the interface offline.
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
     * @note Kvaser filtering is implemented at the hardware level
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
     * @return String describing the Kvaser interface
     */
    std::string getDeviceInfo() const override;
    
private:
    int channel_;                    ///< Kvaser channel number
    int handle_;                     ///< Kvaser handle
    bool is_started_;                ///< Interface start state
    uint32_t bitrate_;               ///< Configured bitrate
    std::vector<CanFilter> filters_; ///< Active filters
    
    /**
     * @brief Convert bitrate to Kvaser bitrate constant
     * @param bitrate The bitrate in bps
     * @return Kvaser bitrate constant
     * 
     * @throws CanBusException if bitrate is not supported
     */
    int bitrateToKvaserConstant(uint32_t bitrate) const;
    
    /**
     * @brief Check Kvaser status and throw exception if error
     * @param operation The operation being performed (for error messages)
     * @param status The Kvaser status to check
     * 
     * @throws CanBusException if status indicates an error
     */
    void checkKvaserStatus(const std::string& operation, int status) const;
};

} // namespace unican
