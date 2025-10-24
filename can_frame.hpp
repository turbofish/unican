/**
 * @file can_frame.hpp
 * @brief CAN frame representation
 * @author Tomas Hellström <turbofish@fripost.org>
 * @copyright Copyright (c) 2025 Tomas Hellström <turbofish@fripost.org>
 * @date 2025
 * @note This file is part of the unican project.
 * @note This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#pragma once
#include <cstdint>
#include <array>
#include <chrono>
#include <string>

// Check for std::span support
#if __cpp_lib_span >= 202002L
#include <span>
#else
// Fallback span implementation for older compilers
#include <iterator>
namespace std {
    template<typename T>
    class span {
    public:
        constexpr span(T* data, size_t size) : data_(data), size_(size) {}
        constexpr T* data() const { return data_; }
        constexpr size_t size() const { return size_; }
        constexpr T& operator[](size_t idx) const { return data_[idx]; }
        constexpr T* begin() const { return data_; }
        constexpr T* end() const { return data_ + size_; }
    private:
        T* data_;
        size_t size_;
    };
}
#endif

namespace unican {

/// Maximum Data Length Code for CAN FD frames
#define CANFD_MAX_DLC 64

/**
 * @class CanFrame
 * @brief Represents a CAN (Controller Area Network) frame with support for both CAN 2.0 and CAN FD
 * 
 * This class provides a complete representation of a CAN frame including:
 * - Frame ID (11-bit or 29-bit extended)
 * - Data payload (up to 64 bytes for CAN FD)
 * - Frame type flags (extended, RTR, CAN FD)
 * - Timestamp information
 * 
 * The class supports both traditional CAN 2.0 (up to 8 bytes) and CAN FD (up to 64 bytes)
 * frames, with automatic detection of frame type based on data length.
 * 
 * @example
 * ```cpp
 * // Create a standard CAN frame
 * CanFrame frame(0x123, std::span<const uint8_t>{data, 8});
 * 
 * // Create an extended CAN frame
 * CanFrame extFrame(0x12345678, std::span<const uint8_t>{data, 8}, true);
 * 
 * // Use builder pattern
 * CanFrame frame = CanFrame()
 *     .setId(0x123)
 *     .setData(std::span<const uint8_t>{data, 8})
 *     .setExtended(false);
 * ```
 */
class CanFrame {
public:
    /**
     * @brief Default constructor
     * 
     * Creates an empty CAN frame with default values:
     * - ID: 0
     * - DLC: 0
     * - Extended: false
     * - RTR: false
     * - CAN FD: false
     */
    CanFrame() = default;

    /**
     * @brief Constructs a CAN frame with specified parameters
     * 
     * @param id The CAN frame identifier (11-bit for standard, 29-bit for extended)
     * @param data The data payload as a span of bytes
     * @param extended Whether this is an extended (29-bit) frame (default: false)
     * @param rtr Whether this is a Remote Transmission Request frame (default: false)
     * 
     * @note The frame type (CAN 2.0 vs CAN FD) is automatically determined based on data length
     * @note Data length is limited to CANFD_MAX_DLC (64 bytes) for CAN FD frames
     * 
     * @throws std::invalid_argument if data length exceeds maximum allowed
     */
    CanFrame(uint32_t id, std::span<const uint8_t> data, 
             bool extended = false, bool rtr = false);

    /**
     * @brief Default destructor
     */
    ~CanFrame() = default;

    /**
     * @brief Get the CAN frame identifier
     * @return The frame ID (11-bit for standard, 29-bit for extended frames)
     */
    uint32_t id() const noexcept { return id_; }
    
    /**
     * @brief Get the Data Length Code (DLC)
     * @return The number of data bytes in the frame (0-64)
     */
    uint8_t dlc() const noexcept { return dlc_; }

    /**
     * @brief Check if this is an extended frame
     * @return true if this is a 29-bit extended frame, false for 11-bit standard frame
     */
    bool isExtended() const noexcept { return is_extended_; }
    
    /**
     * @brief Check if this is a Remote Transmission Request (RTR) frame
     * @return true if this is an RTR frame, false otherwise
     */
    bool isRtr() const noexcept { return is_rtr_; }
    
    /**
     * @brief Check if this is a CAN FD frame
     * @return true if this is a CAN FD frame (data length > 8), false for CAN 2.0
     */
    bool isFd() const noexcept { return is_fd_; }
    
    /**
     * @brief Get the data payload as a span
     * @return A span containing the frame data (size determined by DLC)
     * 
     * @note The returned span is valid only as long as this CanFrame object exists
     */
    std::span<const uint8_t> data() const noexcept { 
        return {data_.data(), dlc_}; 
    }

    // Setters (builder pattern för fluent API)
    /**
     * @brief Set the CAN frame identifier
     * @param id The frame ID (11-bit for standard, 29-bit for extended)
     * @return Reference to this object for method chaining
     */
    CanFrame& setId(uint32_t id) { id_ = id; return *this; }
    
    /**
     * @brief Set the data payload
     * @param data The data payload as a span of bytes
     * @return Reference to this object for method chaining
     * 
     * @note The DLC is automatically updated based on data size
     * @note CAN FD status is automatically determined based on data length
     * 
     * @throws std::invalid_argument if data length exceeds maximum allowed
     */
    CanFrame& setData(std::span<const uint8_t> data);
    
    /**
     * @brief Set the extended frame flag
     * @param ext true for 29-bit extended frame, false for 11-bit standard frame
     * @return Reference to this object for method chaining
     */
    CanFrame& setExtended(bool ext) { is_extended_ = ext; return *this; }
    
    /**
     * @brief Set the Remote Transmission Request flag
     * @param rtr true for RTR frame, false for data frame
     * @return Reference to this object for method chaining
     */
    CanFrame& setRtr(bool rtr) { is_rtr_ = rtr; return *this; }
        
    // Utility
    /**
     * @brief Convert the CAN frame to a human-readable string
     * @return A string representation of the frame in format: "ID:0x123 DLC:8 [data]"
     * 
     * @example
     * ```cpp
     * CanFrame frame(0x123, {0x01, 0x02, 0x03, 0x04});
     * std::cout << frame.toString(); // Output: "ID:0x123 DLC:4 [01 02 03 04]"
     * ```
     */
    std::string toString() const;

private:
    /// CAN frame identifier (11-bit or 29-bit)
    uint32_t id_ = 0;
    
    /// Data Length Code - number of data bytes (0-64)
    uint8_t dlc_ = 0;
    
    /// Frame data payload (up to 64 bytes for CAN FD)
    std::array<uint8_t, CANFD_MAX_DLC> data_{};
    
    /// Extended frame flag (29-bit ID vs 11-bit ID)
    bool is_extended_ = false;
    
    /// Remote Transmission Request flag
    bool is_rtr_ = false;
    
    /// CAN FD frame flag (determined by data length > 8)
    bool is_fd_ = false;
    
    /// Frame timestamp (microseconds since epoch)
    std::chrono::microseconds timestamp_{};
};

} // namespace unican
