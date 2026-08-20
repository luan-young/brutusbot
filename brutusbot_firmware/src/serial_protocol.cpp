#include "brutusbot_firmware/serial_protocol.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

namespace brutusbot_firmware {
namespace serial_protocol {
namespace {
constexpr std::size_t kHeaderSize = 3;
constexpr std::size_t kFrameOverhead = 1 + 1 + 1 + 1;
constexpr std::size_t kMaxFrameSize = kHeaderSize + kMaxPayloadSize + 1;

std::uint8_t checksum_byte(std::uint8_t value) { return static_cast<std::uint8_t>(value & 0xFFu); }

}  // namespace

bool FramedProtocol::parse_byte(uint8_t byte, Frame* frame) {
    if (frame == nullptr) {
        return false;
    }

    switch (state_) {
        case ParseState::kWaitingStart:
            if (byte == kStartByte) {
                state_ = ParseState::kReadingPayloadSize;
                payload_.clear();
                frame->code = 0;
                frame->data.clear();
            }
            break;

        case ParseState::kReadingPayloadSize:
            if (byte > kMaxPayloadSize) {
                reset();
                return false;
            }
            expected_payload_size_ = byte;
            state_ = ParseState::kReadingSizeChecksum;
            break;

        case ParseState::kReadingSizeChecksum: {
            const uint8_t expected_checksum = compute_size_checksum(expected_payload_size_);
            if (byte != expected_checksum) {
                reset();
                return false;
            }
            payload_.clear();
            payload_.reserve(expected_payload_size_);
            state_ = expected_payload_size_ == 0 ? ParseState::kReadingChecksum : ParseState::kReadingPayload;
            break;
        }

        case ParseState::kReadingPayload:
            payload_.push_back(byte);
            if (payload_.size() == expected_payload_size_) {
                state_ = ParseState::kReadingChecksum;
            }
            break;

        case ParseState::kReadingChecksum: {
            const uint8_t expected_checksum = compute_message_checksum(expected_payload_size_, payload_);
            if (byte != expected_checksum) {
                reset();
                return false;
            }

            if (payload_.empty()) {
                frame->code = 0;
                frame->data.clear();
            } else {
                frame->code = payload_[0];
                frame->data.assign(payload_.begin() + 1, payload_.end());
            }

            reset();
            return true;
        }
    }

    return false;
}

void FramedProtocol::reset() {
    state_ = ParseState::kWaitingStart;
    expected_payload_size_ = 0;
    payload_.clear();
}

FrameResult FramedProtocol::handle_frame(const Frame& frame) const {
    const MessageCode code = static_cast<MessageCode>(frame.code);
    switch (code) {
        case MessageCode::kVelocityReplyOk:
            if (!frame.data.empty()) {
                return Error{"Velocity reply OK frame has unexpected payload"};
            }
            return SetVelocityOk{};

        case MessageCode::kStateReply: {
            if (frame.data.size() < 16) {
                return Error{"Received malformed state reply with insufficient payload size"};
            }

            StateReply result;
            if (!unpack_float(frame.data, 0, &result.left_position) ||
                !unpack_float(frame.data, 4, &result.left_velocity) ||
                !unpack_float(frame.data, 8, &result.right_position) ||
                !unpack_float(frame.data, 12, &result.right_velocity)) {
                return Error{"Failed to decode state reply payload"};
            }

            return result;
        }

        default:
            return Error{std::string("Unexpected message code: ") + std::to_string(static_cast<int>(code))};
    }
}

std::vector<uint8_t> FramedProtocol::encode_frame(uint8_t code, const std::vector<uint8_t>& data) const {
    std::vector<uint8_t> payload;
    payload.reserve(1 + data.size());
    payload.push_back(code);
    payload.insert(payload.end(), data.begin(), data.end());

    const std::size_t payload_size = payload.size();
    if (payload_size > kMaxPayloadSize) {
        return {};
    }

    std::vector<uint8_t> frame;
    frame.reserve(kFrameOverhead + payload_size);
    frame.push_back(kStartByte);
    frame.push_back(static_cast<uint8_t>(payload_size));
    frame.push_back(compute_size_checksum(payload_size));
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(compute_message_checksum(payload_size, payload));
    return frame;
}

std::vector<uint8_t> FramedProtocol::encode_frame(MessageCode code, const std::vector<uint8_t>& data) const {
    return encode_frame(static_cast<uint8_t>(code), data);
}

std::vector<uint8_t> FramedProtocol::encode_wheel_velocity_command(float left_velocity, float right_velocity) const {
    std::vector<uint8_t> data;
    const auto left_bytes = pack_float(left_velocity);
    const auto right_bytes = pack_float(right_velocity);
    data.reserve(left_bytes.size() + right_bytes.size());
    data.insert(data.end(), left_bytes.begin(), left_bytes.end());
    data.insert(data.end(), right_bytes.begin(), right_bytes.end());
    return encode_frame(MessageCode::kSetWheelVelocity, data);
}

std::vector<uint8_t> FramedProtocol::encode_state_request() const {
    return encode_frame(MessageCode::kRequestState, {});
}

std::vector<uint8_t> FramedProtocol::pack_float(float value) {
    std::vector<uint8_t> bytes(4);
    std::memcpy(bytes.data(), &value, sizeof(value));
    return bytes;
}

bool FramedProtocol::unpack_float(const std::vector<uint8_t>& data, std::size_t offset, float* value) {
    if (value == nullptr || offset + sizeof(float) > data.size()) {
        return false;
    }

    std::memcpy(value, data.data() + offset, sizeof(float));
    return true;
}

uint8_t FramedProtocol::compute_size_checksum(std::size_t payload_size) {
    return checksum_byte(static_cast<uint8_t>(payload_size));
}

uint8_t FramedProtocol::compute_message_checksum(std::size_t payload_size, const std::vector<uint8_t>& payload) {
    uint8_t checksum = 0;
    checksum = static_cast<uint8_t>(checksum + static_cast<uint8_t>(payload_size));
    for (const auto byte : payload) {
        checksum = static_cast<uint8_t>(checksum + byte);
    }
    return checksum;
}

}  // namespace serial_protocol
}  // namespace brutusbot_firmware
