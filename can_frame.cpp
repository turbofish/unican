#include "can_frame.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace unican
{
    CanFrame::CanFrame(uint32_t id, std::span<const uint8_t> data,
                       bool extended, bool rtr)
        : id_(id), is_extended_(extended), is_rtr_(rtr), is_fd_(false), timestamp_(0)
    {
        setData(data);
    }

    CanFrame &CanFrame::setData(std::span<const uint8_t> data)
    {
        size_t max_len = is_fd_ ? 64 : 8;
        if (data.size() > max_len)
        {
            throw std::invalid_argument(
                "Data length exceeds maximum for " +
                std::string(is_fd_ ? "CAN FD" : "CAN") +
                " frame");
        }

        dlc_ = static_cast<uint8_t>(data.size());
        std::copy(data.begin(), data.end(), data_.begin());

        return *this;
    }

    std::string CanFrame::toString() const
    {
        std::ostringstream oss;

        // ID (hex format)
        oss << "0x" << std::hex << std::setfill('0');
        if (is_extended_)
        {
            oss << std::setw(8) << id_;
        }
        else
        {
            oss << std::setw(3) << id_;
        }

        // Flags
        if (is_extended_)
        {
            oss << " [EXT]";
        }
        if (is_rtr_)
        {
            oss << " [RTR]";
        }
        if (is_fd_)
        {
            oss << " [FD]";
        }

        // DLC
        oss << "DLC:" << std::dec << static_cast<int>(dlc_);

        // Data bytes
        if (!is_rtr_ && dlc_ > 0)
        {
            oss << " [";
            for (size_t i = 0; i < dlc_; ++i)
            {
                if (i > 0)
                    oss << " ";
                oss << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<int>(data_[i]);
            }
            oss << "]";
        }

        // Timestamp if present
        if (timestamp_.count() > 0)
        {
            oss << " Time: " << std::dec << timestamp_.count() << "us";
        }

        return oss.str();
    }

} // namespace unican