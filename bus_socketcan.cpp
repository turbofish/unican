/**
 * @file interface_socketcan.cpp
 * @brief SocketCAN backend implementation for Linux CAN interfaces
 * @author Tomas Hellström <turbofish@fripost.org>
 * @copyright Copyright (c) 2025 Tomas Hellström <turbofish@fripost.org>
 * 
 * @note This file is part of the unican project.
 * @note This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This implementation provides SocketCAN support for Linux systems,
 * allowing communication with CAN devices through the standard Linux
 * SocketCAN interface.
 */

#include "bus_socketcan.hpp"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <chrono>
#include <thread>

namespace unican {

SocketCanBus::SocketCanBus(const std::string& interface_name)
    : interface_name_(interface_name)
    , socket_fd_(-1)
    , is_started_(false)
    , bitrate_(0)
{
    // Socket will be created when start() is called
}
    
SocketCanBus::~SocketCanBus() {
    if (is_started_) {
        stop();
    }
    if (socket_fd_ != -1) {
        close(socket_fd_);
    }
}
    
void SocketCanBus::setBitrate(uint32_t bitrate) {
    bitrate_ = bitrate;
    // Note: SocketCAN bitrate is typically set at the system level
    // using 'ip link set can0 type can bitrate 500000'
}
    
void SocketCanBus::start() {
    if (is_started_) {
        return; // Already started
    }
    
    // Create CAN socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        throw CanBusException("Failed to create CAN socket: " + std::string(strerror(errno)));
    }
    
    // Get interface index
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, interface_name_.c_str());
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        throw CanBusException("Interface " + interface_name_ + " not found: " + std::string(strerror(errno)));
    }
    
    // Bind socket to interface
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        throw CanBusException("Failed to bind to interface " + interface_name_ + ": " + std::string(strerror(errno)));
    }
    
    is_started_ = true;
}
void SocketCanBus::stop() {
    if (!is_started_) {
        return; // Already stopped
    }
    
    if (socket_fd_ != -1) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    
    is_started_ = false;
}

void SocketCanBus::send(const CanFrame& frame) {
    if (!is_started_) {
        throw CanBusException("Interface not started");
    }
    
    struct can_frame can_frame;
    can_frame.can_id = frame.id();
    can_frame.can_dlc = frame.dlc();
    
    // Set extended frame flag
    if (frame.isExtended()) {
        can_frame.can_id |= CAN_EFF_FLAG;
    }
    
    // Set RTR flag
    if (frame.isRtr()) {
        can_frame.can_id |= CAN_RTR_FLAG;
    }
    
    // Copy data
    const auto data = frame.data();
    for (size_t i = 0; i < frame.dlc() && i < CAN_MAX_DLEN; ++i) {
        can_frame.data[i] = data[i];
    }
    
    // Send frame
    ssize_t bytes_sent = write(socket_fd_, &can_frame, sizeof(can_frame));
    if (bytes_sent < 0) {
        throw CanBusException("Failed to send frame: " + std::string(strerror(errno)));
    }
}

std::optional<CanFrame> SocketCanBus::receive(std::chrono::milliseconds timeout) {
    if (!is_started_) {
        throw CanBusException("Interface not started");
    }
    
    // Set socket to non-blocking for timeout support
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
    
    struct can_frame can_frame;
    ssize_t bytes_received;
    
    if (timeout.count() == -1) {
        // Blocking receive
        fcntl(socket_fd_, F_SETFL, flags); // Restore blocking
        bytes_received = read(socket_fd_, &can_frame, sizeof(can_frame));
    } else {
        // Non-blocking receive with timeout
        auto start_time = std::chrono::steady_clock::now();
        do {
            bytes_received = read(socket_fd_, &can_frame, sizeof(can_frame));
            if (bytes_received > 0) {
                break;
            }
            
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                break;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } while (std::chrono::steady_clock::now() - start_time < timeout);
    }
    
    // Restore original flags
    fcntl(socket_fd_, F_SETFL, flags);
    
    if (bytes_received <= 0) {
        return std::nullopt;
    }
    
    // Convert to CanFrame
    uint32_t id = can_frame.can_id & CAN_SFF_MASK;
    bool extended = (can_frame.can_id & CAN_EFF_FLAG) != 0;
    bool rtr = (can_frame.can_id & CAN_RTR_FLAG) != 0;
    
    if (extended) {
        id = can_frame.can_id & CAN_EFF_MASK;
    }
    
    // Create CanFrame
    std::array<uint8_t, CANFD_MAX_DLC> data{};
    for (int i = 0; i < can_frame.can_dlc && i < CAN_MAX_DLEN; ++i) {
        data[i] = can_frame.data[i];
    }
    
    return CanFrame(id, std::span<const uint8_t>{data.data(), static_cast<size_t>(can_frame.can_dlc)}, extended, rtr);
}

void SocketCanBus::setFilter(const CanFilter& filter) {
    if (!is_started_) {
        throw CanBusException("Interface not started");
    }
    
    struct can_filter can_filter;
    can_filter.can_id = filter.id;
    can_filter.can_mask = filter.mask;
    
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &can_filter, sizeof(can_filter)) < 0) {
        throw CanBusException("Failed to set filter: " + std::string(strerror(errno)));
    }
}

void SocketCanBus::clearFilters() {
    if (!is_started_) {
        return;
    }
    
    // Remove all filters by setting an empty filter list
    struct can_filter filter[0];
    setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filter, 0);
}

bool SocketCanBus::isStarted() const {
    return is_started_;
}

std::string SocketCanBus::getDeviceInfo() const {
    return "SocketCAN interface: " + interface_name_ + " (" + std::to_string(bitrate_) + " bps)";
}

} // namespace unican
