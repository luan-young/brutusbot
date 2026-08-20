#ifndef BRUTUSBOT_FIRMWARE_SERIAL_PROTOCOL_HPP_
#define BRUTUSBOT_FIRMWARE_SERIAL_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace brutusbot_firmware {
namespace serial_protocol {

constexpr uint8_t kStartByte = 0x7E;
constexpr std::size_t kMaxPayloadSize = 32;

enum class MessageCode : uint8_t {
    kSetWheelVelocity = 0x10,
    kRequestState = 0x11,
    kVelocityReplyOk = 0x20,
    kStateReply = 0x21,
};

struct Frame {
    uint8_t code = 0;
    std::vector<uint8_t> data;
};

struct SetVelocityOk {};

struct StateReply {
    float left_position = 0.0f;
    float left_velocity = 0.0f;
    float right_position = 0.0f;
    float right_velocity = 0.0f;
};

struct Error {
    std::string message;
};

using FrameResult = std::variant<SetVelocityOk, StateReply, Error>;

class FramedProtocol {
public:
    bool parse_byte(uint8_t byte, Frame* frame);
    void reset();
    FrameResult handle_frame(const Frame& frame) const;

    std::vector<uint8_t> encode_frame(uint8_t code, const std::vector<uint8_t>& data) const;
    std::vector<uint8_t> encode_frame(MessageCode code, const std::vector<uint8_t>& data) const;
    std::vector<uint8_t> encode_wheel_velocity_command(float left_velocity, float right_velocity) const;
    std::vector<uint8_t> encode_state_request() const;

    static std::vector<uint8_t> pack_float(float value);
    static bool unpack_float(const std::vector<uint8_t>& data, std::size_t offset, float* value);

private:
    static uint8_t compute_size_checksum(std::size_t payload_size);
    static uint8_t compute_message_checksum(std::size_t payload_size, const std::vector<uint8_t>& payload);

    enum class ParseState {
        kWaitingStart,
        kReadingPayloadSize,
        kReadingSizeChecksum,
        kReadingPayload,
        kReadingChecksum,
    };

    ParseState state_{ParseState::kWaitingStart};
    std::size_t expected_payload_size_{0};
    std::vector<uint8_t> payload_;
};

}  // namespace serial_protocol
}  // namespace brutusbot_firmware

#endif  // BRUTUSBOT_FIRMWARE_SERIAL_PROTOCOL_HPP_
