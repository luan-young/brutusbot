#include <libserial/SerialStream.h>

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class SerialTestNode : public rclcpp::Node {
public:
    SerialTestNode() : Node("test_serial"), serial_port_("/dev/ttyUSB0"), timeout_ms_(2000), message_("arduino_ping") {
        // declare_parameter("serial_port", serial_port_);
        // declare_parameter("timeout_ms", timeout_ms_);
        // declare_parameter("message", message_);

        // get_parameter("serial_port", serial_port_);
        // get_parameter("timeout_ms", timeout_ms_);
        // get_parameter("message", message_);

        timer_ = create_wall_timer(5s, std::bind(&SerialTestNode::on_timer, this));
    }

private:
    void on_timer() {
        if (!open_serial_port()) {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port '%s'", serial_port_.c_str());
            return;
        }

        const std::string expected_message = message_;
        const std::string to_send = expected_message + "\n";

        RCLCPP_INFO(get_logger(), "Startin");
        serial_stream_ << to_send;
        serial_stream_.flush();
        RCLCPP_INFO(get_logger(), "Sent");

        std::string reply;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);

        while (std::chrono::steady_clock::now() < deadline) {
            if (serial_stream_.IsDataAvailable()) {
                RCLCPP_INFO(get_logger(), "Got data");
                if (std::getline(serial_stream_, reply)) {
                    RCLCPP_INFO(get_logger(), "Read data");
                    break;
                }
            }
            std::this_thread::sleep_for(1ms);
        }

        if (reply.empty()) {
            RCLCPP_WARN(get_logger(), "Serial timeout waiting for reply after %u ms", timeout_ms_);
            return;
        }

        if (reply == expected_message) {
            RCLCPP_INFO(get_logger(), "Received expected reply: '%s'", reply.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "Received unexpected reply: '%s'", reply.c_str());
        }
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
