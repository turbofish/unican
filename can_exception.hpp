/**
 * @file can_exception.hpp
 * @brief CAN-specific exception classes
 * @author Tomas Hellström <turbofish@fripost.org>
 * @copyright Copyright (c) 2025 Tomas Hellström <turbofish@fripost.org>
 * 
 * @note This file is part of the unican project.
 * @note This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This file defines exception classes specific to CAN communication,
 * providing detailed error information for debugging and error handling.
 */

#pragma once

#include <stdexcept>
#include <string>

namespace unican {

/**
 * @class CanException
 * @brief Base exception class for CAN communication errors
 * 
 * This exception is thrown when CAN-specific errors occur during
 * communication, configuration, or hardware operations.
 * 
 * @example
 * ```cpp
 * try {
 *     canBus->send(frame);
 * } catch (const CanException& e) {
 *     std::cerr << "CAN Error: " << e.what() << std::endl;
 * }
 * ```
 */
class CanException : public std::runtime_error {
public:
    /**
     * @brief Construct a CAN exception with a message
     * @param message The error message describing what went wrong
     */
    explicit CanException(const std::string& message)
        : std::runtime_error(message) {}
    
    /**
     * @brief Construct a CAN exception with a message
     * @param message The error message describing what went wrong
     */
    explicit CanException(const char* message)
        : std::runtime_error(message) {}
};

/**
 * @class CanBusException
 * @brief Exception thrown when bus-related errors occur
 * 
 * This exception is thrown when errors occur that are specifically
 * related to the CAN bus state or configuration.
 */
class CanBusException : public CanException {
public:
    /**
     * @brief Construct a CAN bus exception with a message
     * @param message The error message describing the bus error
     */
    explicit CanBusException(const std::string& message)
        : CanException(message) {}
    
    /**
     * @brief Construct a CAN bus exception with a message
     * @param message The error message describing the bus error
     */
    explicit CanBusException(const char* message)
        : CanException(message) {}
};

/**
 * @class CanFrameException
 * @brief Exception thrown when frame-related errors occur
 * 
 * This exception is thrown when errors occur during frame
 * creation, validation, or transmission.
 */
class CanFrameException : public CanException {
public:
    /**
     * @brief Construct a CAN frame exception with a message
     * @param message The error message describing the frame error
     */
    explicit CanFrameException(const std::string& message)
        : CanException(message) {}
    
    /**
     * @brief Construct a CAN frame exception with a message
     * @param message The error message describing the frame error
     */
    explicit CanFrameException(const char* message)
        : CanException(message) {}
};

} // namespace unican
