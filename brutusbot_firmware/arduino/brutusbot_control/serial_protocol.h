#include <Arduino.h>

constexpr uint8_t kStartByte = 0x7E;
constexpr uint8_t kMaxDataSize = 32;
constexpr uint8_t kMaxPayloadSize = kMaxDataSize + 1;  // +1 for the message code

enum MessageCode : uint8_t {
    kSetWheelVelocity = 0x10,
    kRequestState = 0x11,
    kVelocityReplyOk = 0x20,
    kStateReply = 0x21,
};

struct Frame {
    uint8_t code = 0;
    uint8_t data[kMaxDataSize] = {0};
    uint8_t data_size = 0;
};

struct ParserState {
    enum State : uint8_t {
        kWaitingStart,
        kReadingPayloadSize,
        kReadingSizeChecksum,
        kReadingPayload,
        kReadingChecksum,
    } state = kWaitingStart;

    uint8_t expected_payload_size = 0;
    uint8_t payload_index = 0;
    uint8_t payload[kMaxPayloadSize] = {0};
};

uint8_t compute_size_checksum(uint8_t payload_size) { return payload_size & 0xFF; }

uint8_t compute_message_checksum(uint8_t payload_size, const uint8_t* payload, uint8_t payload_size_bytes) {
    uint8_t checksum = payload_size;
    for (uint8_t i = 0; i < payload_size_bytes; ++i) {
        checksum += payload[i];
    }
    return checksum;
}

void reset_parser(ParserState& parser) {
    parser.state = ParserState::kWaitingStart;
    parser.expected_payload_size = 0;
    parser.payload_index = 0;
}

bool parse_byte(uint8_t byte, ParserState& parser, Frame& frame) {
    switch (parser.state) {
        case ParserState::kWaitingStart:
            if (byte == kStartByte) {
                parser.state = ParserState::kReadingPayloadSize;
                frame.code = 0;
                frame.data_size = 0;
            }
            break;

        case ParserState::kReadingPayloadSize:
            if (byte > kMaxPayloadSize || byte == 0) {
                reset_parser(parser);
                return false;
            }
            parser.expected_payload_size = byte;
            parser.state = ParserState::kReadingSizeChecksum;
            break;

        case ParserState::kReadingSizeChecksum: {
            const uint8_t expected_checksum = compute_size_checksum(parser.expected_payload_size);
            if (byte != expected_checksum) {
                reset_parser(parser);
                return false;
            }
            parser.payload_index = 0;
            parser.state = ParserState::kReadingPayload;
            break;
        }

        case ParserState::kReadingPayload:
            parser.payload[parser.payload_index++] = byte;
            if (parser.payload_index == parser.expected_payload_size) {
                parser.state = ParserState::kReadingChecksum;
            }
            break;

        case ParserState::kReadingChecksum: {
            const uint8_t expected_checksum =
                compute_message_checksum(parser.expected_payload_size, parser.payload, parser.expected_payload_size);
            if (byte != expected_checksum) {
                reset_parser(parser);
                return false;
            }

            frame.code = parser.payload[0];
            frame.data_size = static_cast<uint8_t>(parser.expected_payload_size - 1);
            for (uint8_t i = 0; i < frame.data_size; ++i) {
                frame.data[i] = parser.payload[i + 1];
            }

            reset_parser(parser);
            return true;
        }
    }

    return false;
}

void send_frame(const uint8_t* payload, uint8_t payload_size) {
    Serial.write(kStartByte);
    Serial.write(payload_size);
    Serial.write(compute_size_checksum(payload_size));
    Serial.write(payload, payload_size);
    Serial.write(compute_message_checksum(payload_size, payload, payload_size));
}

void send_velocity_reply_ok() {
    uint8_t payload[1] = {kVelocityReplyOk};
    send_frame(payload, sizeof(payload));
}

void send_state_reply(float left_position, float left_velocity, float right_position, float right_velocity) {
    uint8_t payload[sizeof(uint8_t) + 4 * sizeof(float)];
    payload[0] = kStateReply;
    memcpy(payload + sizeof(uint8_t), &left_position, sizeof(left_position));
    memcpy(payload + sizeof(uint8_t) + sizeof(float), &left_velocity, sizeof(left_velocity));
    memcpy(payload + sizeof(uint8_t) + 2 * sizeof(float), &right_position, sizeof(right_position));
    memcpy(payload + sizeof(uint8_t) + 3 * sizeof(float), &right_velocity, sizeof(right_velocity));
    send_frame(payload, sizeof(payload));
}