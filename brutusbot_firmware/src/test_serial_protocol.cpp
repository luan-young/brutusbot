#include <libserial/SerialStream.h>

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <variant>

#include "brutusbot_firmware/serial_protocol.hpp"

using namespace std::chrono_literals;
using namespace brutusbot_firmware;

class SerialTestNode : public rclcpp::Node {
public:
    SerialTestNode() : Node("test_serial"), serial_port_("/dev/ttyUSB0"), timeout_ms_(2000), message_("arduino_ping") {
        // declare_parameter("serial_port", serial_port_);
        // declare_parameter("timeout_ms", timeout_ms_);
        // declare_parameter("message", message_);

        // get_parameter("serial_port", serial_port_);
        // get_parameter("timeout_ms", timeout_ms_);
        // get_parameter("message", message_);

        timer_ = create_wall_timer(1000ms, std::bind(&SerialTestNode::on_timer, this));
    }

private:
    bool send_frame(const std::vector<uint8_t>& frame) {
        if (frame.empty()) {
            return false;
        }

        try {
            serial_stream_.write(reinterpret_cast<const char*>(frame.data()),
                                 static_cast<std::streamsize>(frame.size()));
            serial_stream_.flush();
        } catch (const std::exception& ex) {
            RCLCPP_ERROR(get_logger(), "Failed to send serial frame: %s", ex.what());
            return false;
        }

        return true;
    }

    bool process_serial_bytes(serial_protocol::MessageCode expected_code) {
        while (serial_stream_.IsDataAvailable()) {
            char byte = 0;
            serial_stream_.get(byte);
            serial_protocol::Frame frame;
            if (serial_protocol_.parse_byte(static_cast<uint8_t>(byte), &frame)) {
                RCLCPP_INFO(get_logger(), "Received protocol frame");
                const auto result = serial_protocol_.handle_frame(frame);
                if (const auto error = std::get_if<serial_protocol::Error>(&result)) {
                    RCLCPP_WARN(get_logger(), "Protocol error: %s", error->message.c_str());
                    return false;
                }

                if (expected_code == serial_protocol::MessageCode::kVelocityReplyOk &&
                    std::holds_alternative<serial_protocol::SetVelocityOk>(result)) {
                    RCLCPP_INFO(get_logger(), "Received velocity reply OK");
                    return true;
                }

                if (expected_code == serial_protocol::MessageCode::kStateReply) {
                    if (const auto state = std::get_if<serial_protocol::StateReply>(&result)) {
                        RCLCPP_INFO(get_logger(),
                                    "Received state reply: left_position=%.3f, left_velocity=%.3f, "
                                    "right_position=%.3f, right_velocity=%.3f",
                                    state->left_position, state->left_velocity, state->right_position,
                                    state->right_velocity);
                        return true;
                    }
                }

                RCLCPP_WARN(get_logger(), "Received unexpected protocol result for expected code %d",
                            static_cast<int>(expected_code));
                return false;
            }
        }
        return false;
    }

    void on_timer() {
        static float left_velocity_cmd = 0.0f;
        static float right_velocity_cmd = 0.0f;

        if (!open_serial_port()) {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port '%s'", serial_port_.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "Starting");

        const auto set_velocity_request =
            serial_protocol_.encode_wheel_velocity_command(left_velocity_cmd, right_velocity_cmd);
        if (!send_frame(set_velocity_request)) {
            RCLCPP_ERROR(get_logger(), "Failed to send wheel velocity command");
            return;
        }
        RCLCPP_INFO(get_logger(), "Sent set velocity command");
        if (!process_serial_bytes(serial_protocol::MessageCode::kVelocityReplyOk)) {
            RCLCPP_ERROR(get_logger(), "Failed to receive velocity reply");
        }

        const auto state_request = serial_protocol_.encode_state_request();
        if (!send_frame(state_request)) {
            RCLCPP_ERROR(get_logger(), "Failed to send state request");
            return;
        }
        RCLCPP_INFO(get_logger(), "Sent state request");
        if (!process_serial_bytes(serial_protocol::MessageCode::kStateReply)) {
            RCLCPP_ERROR(get_logger(), "Failed to receive state reply");
        }

        left_velocity_cmd += 0.1f;
        right_velocity_cmd -= 0.1f;
    }

    bool open_serial_port() {
        if (serial_stream_.IsOpen()) {
            return true;
        }

        try {
            serial_stream_.Open(serial_port_, std::ios_base::in | std::ios_base::out);
            serial_stream_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            serial_stream_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_stream_.SetParity(LibSerial::Parity::PARITY_NONE);
            serial_stream_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            serial_stream_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            serial_stream_.SetVMin(0);
            serial_stream_.SetVTime(static_cast<short>((timeout_ms_ + 99) / 100));
            serial_stream_.FlushIOBuffers();
        } catch (const std::exception& ex) {
            RCLCPP_ERROR(get_logger(), "Serial open failed: %s", ex.what());
            return false;
        }

        return serial_stream_.IsOpen();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::string serial_port_;
    serial_protocol::FramedProtocol serial_protocol_;
    int timeout_ms_;
    std::string message_;
    LibSerial::SerialStream serial_stream_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialTestNode>());
    rclcpp::shutdown();
    return 0;
}
