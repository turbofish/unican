/**
 * @file interface_kvaser.cpp
 * @brief Kvaser CANlib backend implementation for Kvaser CAN interfaces
 * @author Tomas Hellström <turbofish@fripost.org>
 * @copyright Copyright (c) 2025 Tomas Hellström <turbofish@fripost.org>
 * 
 * @note This file is part of the unican project.
 * @note This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This implementation provides Kvaser CANlib support for Kvaser CAN interfaces,
 * allowing communication with Kvaser hardware through the Kvaser CANlib API.
 */

#include "bus_kvaser.hpp"
#include <canlib.h>
#include <canstat.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <algorithm>
#include <optional>

namespace unican {

KvaserCanBus::KvaserCanBus(int channel)
    : channel_(channel)
    , handle_(canINVALID_HANDLE)
    , is_started_(false)
    , bitrate_(0)
{
    if (channel < 0) {
        throw CanBusException("Invalid channel number: " + std::to_string(channel));
    }
}

KvaserCanBus::~KvaserCanBus() {
    if (is_started_) {
        stop();
    }
    if (handle_ != canINVALID_HANDLE) {
        canClose(handle_);
    }
}

void KvaserCanBus::setBitrate(uint32_t bitrate) {
    bitrate_ = bitrate;
    // Bitrate will be applied when start() is called
}

void KvaserCanBus::start() {
    if (is_started_) {
        return; // Already started
    }
    
    // Initialize CANlib
    canInitializeLibrary();
    
    // Open channel
    handle_ = canOpenChannel(channel_, canOPEN_EXCLUSIVE | canOPEN_ACCEPT_VIRTUAL);
    if (handle_ < 0) {
        throw CanBusException("Failed to open Kvaser channel " + std::to_string(channel_) + 
                             ": " + std::to_string(handle_));
    }
    
    // Set bitrate
    int kvaserBitrate = bitrateToKvaserConstant(bitrate_);
    int status = canSetBusParams(handle_, kvaserBitrate, 0, 0, 0, 0, 0);
    checkKvaserStatus("canSetBusParams", status);
    
    // Go on bus
    status = canBusOn(handle_);
    checkKvaserStatus("canBusOn", status);
    
    is_started_ = true;
}

void KvaserCanBus::stop() {
    if (!is_started_) {
        return; // Already stopped
    }
    
    if (handle_ != canINVALID_HANDLE) {
        canBusOff(handle_);
        canClose(handle_);
        handle_ = canINVALID_HANDLE;
    }
    
    is_started_ = false;
}

void KvaserCanBus::send(const CanFrame& frame) {
    if (!is_started_) {
        throw CanBusException("Interface not started");
    }
    
    // Prepare data array
    unsigned char data[8];
    const auto frameData = frame.data();
    size_t dataSize = std::min(static_cast<size_t>(frame.dlc()), static_cast<size_t>(8));
    
    for (size_t i = 0; i < dataSize; ++i) {
        data[i] = frameData[i];
    }
    
    // Set flags
    unsigned int flags = 0;
    if (frame.isExtended()) {
        flags |= canMSG_EXT;
    }
    if (frame.isRtr()) {
        flags |= canMSG_RTR;
    }
    
    // Send frame
    int status = canWrite(handle_, frame.id(), data, frame.dlc(), flags);
    checkKvaserStatus("canWrite", status);
    
    // Wait for transmission to complete
    status = canWriteSync(handle_, 1000); // 1 second timeout
    checkKvaserStatus("canWriteSync", status);
}

std::optional<CanFrame> KvaserCanBus::receive(std::chrono::milliseconds timeout) {
    if (!is_started_) {
        throw CanBusException("Interface not started");
    }
    
    long id;
    unsigned char data[8];
    unsigned int dlc;
    unsigned int flags;
    unsigned long timestamp;
    
    // Convert timeout to milliseconds
    unsigned long waitTime = (timeout.count() == -1) ? 
        static_cast<unsigned long>(-1) : static_cast<unsigned long>(timeout.count());
    
    int status = canReadWait(handle_, &id, data, &dlc, &flags, &timestamp, waitTime);
    
    if (status != canOK) {
        if (status == canERR_NOMSG) {
            return std::nullopt; // Timeout
        }
        checkKvaserStatus("canReadWait", status);
    }
    
    // Check for error frames
    if (flags & canMSG_ERROR_FRAME) {
        return std::nullopt; // Skip error frames
    }
    
    // Convert to CanFrame
    bool extended = (flags & canMSG_EXT) != 0;
    bool rtr = (flags & canMSG_RTR) != 0;
    
    // Create data array
    std::array<uint8_t, CANFD_MAX_DLC> frameData{};
    for (size_t i = 0; i < dlc && i < CANFD_MAX_DLC; ++i) {
        frameData[i] = data[i];
    }
    
    return CanFrame(static_cast<uint32_t>(id), 
                   std::span<const uint8_t>{frameData.data(), dlc}, 
                   extended, rtr);
}

void KvaserCanBus::setFilter(const CanFilter& filter) {
    if (!is_started_) {
        throw CanBusException("Interface not started");
    }
    
    // Store filter for reference
    filters_.push_back(filter);
    
    // Note: Kvaser filtering is typically done at the application level
    // Hardware filtering capabilities vary by device
    // For now, we'll store the filters and apply them in software
}

void KvaserCanBus::clearFilters() {
    filters_.clear();
    // All frames will be received
}

bool KvaserCanBus::isStarted() const {
    return is_started_;
}

std::string KvaserCanBus::getDeviceInfo() const {
    if (!is_started_) {
        return "Kvaser interface: channel " + std::to_string(channel_) + " (not started)";
    }
    
    // Get device information
    char name[256];
    int status = canGetChannelData(channel_, canCHANNELDATA_DEVDESCR_ASCII, name, sizeof(name));
    if (status == canOK) {
        return "Kvaser interface: " + std::string(name) + " (channel " + 
               std::to_string(channel_) + ", " + std::to_string(bitrate_) + " bps)";
    }
    
    return "Kvaser interface: channel " + std::to_string(channel_) + 
           " (" + std::to_string(bitrate_) + " bps)";
}

int KvaserCanBus::bitrateToKvaserConstant(uint32_t bitrate) const {
    switch (bitrate) {
        case 10000:   return canBITRATE_10K;
        case 50000:   return canBITRATE_50K;
        case 100000:  return canBITRATE_100K;
        case 125000:  return canBITRATE_125K;
        case 250000:  return canBITRATE_250K;
        case 500000:  return canBITRATE_500K;
        case 1000000: return canBITRATE_1M;
        default:
            throw CanBusException("Unsupported bitrate: " + std::to_string(bitrate) + " bps");
    }
}

void KvaserCanBus::checkKvaserStatus(const std::string& operation, int status) const {
    if (status != canOK) {
        char errorText[256];
        canGetErrorText((canStatus) status, errorText, sizeof(errorText));
        throw CanBusException(operation + " failed: " + std::string(errorText) + 
                             " (status: " + std::to_string(status) + ")");
    }
}

} // namespace unican
