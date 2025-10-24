/**
 * @file bus_null.hpp
 * @brief Null CAN bus implementation for testing and simulation
 * @author Tomas Hellström <turbofish@fripost.org>
 * @copyright Copyright (c) 2025 Tomas Hellström <turbofish@fripost.org>
 * 
 * @note This file is part of the unican project.
 * @note This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This header defines the NullCanBus class which provides a null implementation
 * of the CAN bus interface for testing and simulation purposes.
 */

#pragma once

#include "can_bus.hpp"
#include "can_exception.hpp"
#include <string>
#include <vector>

namespace unican {

/**
 * @class NullCanBus
 * @brief Null implementation of the CAN bus interface for testing
 * 
 * This class provides a complete implementation of the ICanBus interface
 * that doesn't actually communicate with any hardware. It's useful for:
 * - Unit testing
 * - Simulation
 * - Development when no CAN hardware is available
 * 
 * @note All operations are simulated and don't perform actual CAN communication
 * @note Thread-safe for concurrent access
 * 
 * @example
 * ```cpp
 * // Create null CAN interface for testing
 * auto canBus = std::make_unique<NullCanBus>("test");
 * 
 * // Configure and start
 * canBus->setBitrate(500000);
 * canBus->start();
 * 
 * // Send a frame (simulated)
 * CanFrame frame(0x123, std::span<const uint8_t>{data, 4});
 * canBus->send(frame);  // This will succeed but not actually transmit
 * ```
 */
class NullCanBus : public ICanBus {
public:
    /**
     * @brief Construct a null CAN interface
     * @param channel The channel name (for identification purposes)
     */
    explicit NullCanBus(const std::string& channel);
    
    /**
     * @brief Destructor
     */
    ~NullCanBus() override = default;
    
    /**
     * @brief Set the CAN bus bitrate (simulated)
     * @param bitrate The desired bitrate in bits per second
     */
    void setBitrate(uint32_t bitrate) override;
    
    /**
     * @brief Start the CAN interface (simulated)
     */
    void start() override;
    
    /**
     * @brief Stop the CAN interface (simulated)
     */
    void stop() override;
    
    /**
     * @brief Send a CAN frame (simulated)
     * @param frame The CAN frame to send
     * 
     * @note This method simulates sending but doesn't actually transmit
     * @throws CanBusException if interface is not started
     */
    void send(const CanFrame& frame) override;
    
    /**
     * @brief Receive a CAN frame (simulated)
     * @param timeout Maximum time to wait for a frame
     * @return Always returns empty (no frames received in simulation)
     * 
     * @note This method always returns empty as no frames are received in simulation
     */
    std::optional<CanFrame> receive(std::chrono::milliseconds timeout = std::chrono::milliseconds(-1)) override;
    
    /**
     * @brief Set a hardware filter (simulated)
     * @param filter The filter to apply
     * 
     * @note Filters are stored but not applied to actual hardware
     */
    void setFilter(const CanFilter& filter) override;
    
    /**
     * @brief Clear all hardware filters (simulated)
     */
    void clearFilters() override;
    
    /**
     * @brief Check if interface is started
     * @return true if interface is active, false otherwise
     */
    bool isStarted() const override;
    
    /**
     * @brief Get device information
     * @return String describing the null CAN interface
     */
    std::string getDeviceInfo() const override;
    
private:
    std::string channel_;              ///< Channel name for identification
    bool is_started_;                 ///< Interface start state
    uint32_t bitrate_;                ///< Configured bitrate
    std::vector<CanFilter> filters_;  ///< Stored filters (not applied)
    size_t tx_count_;                 ///< Transmission counter
};

} // namespace unican
