#include "brutusbot_firmware/brutusbot_interface.hpp"

#include <libserial/SerialStream.h>

#include <cstdio>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace brutusbot_firmware {
namespace {
constexpr const char* kLeftJointName = "left_motor_wheel_joint";
constexpr const char* kRightJointName = "right_motor_wheel_joint";
}  // namespace

hardware_interface::CallbackReturn BrutusbotInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (!validate_joint_names()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_commands_.resize(info_.joints.size(), 0.0);
    hw_states_position_.resize(info_.joints.size(), 0.0);
    hw_states_velocity_.resize(info_.joints.size(), 0.0);
    configure_serial_port();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BrutusbotInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!open_serial_port()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BrutusbotInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
    close_serial_port();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BrutusbotInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    curr_read_errors_ = 0;
    curr_write_errors_ = 0;

    if (!serial_stream_.IsOpen()) {
        if (!open_serial_port()) {
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    if (!read_initial_state()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BrutusbotInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    close_serial_port();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BrutusbotInterface::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
    close_serial_port();
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BrutusbotInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                                      &hw_states_position_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
                                      &hw_states_velocity_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BrutusbotInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
    }
    return command_interfaces;
}

hardware_interface::return_type BrutusbotInterface::read(const rclcpp::Time& /*time*/,
                                                         const rclcpp::Duration& /*period*/) {
    if (!serial_stream_.IsOpen()) {
        return hardware_interface::return_type::ERROR;
    }

    const auto state_request = serial_protocol_.encode_state_request();
    if (!send_frame(state_request)) {
        RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"), "Failed to send state request frame");
        return hardware_interface::return_type::ERROR;
    }

    if (!wait_data_available(std::chrono::milliseconds(2))) {
        if (increment_and_check_read_errors()) {
            RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"), "Timeout waiting for state reply from serial port");
            return hardware_interface::return_type::ERROR;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("BrutusbotInterface"),
                        "Timeout waiting for state reply from serial port, retrying...");
            return hardware_interface::return_type::OK;
        }
    }

    auto const result = process_serial_bytes();
    if (std::holds_alternative<serial_protocol::StateReply>(result)) {
        if (!handle_state_reply(std::get<serial_protocol::StateReply>(result))) {
            RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"), "Failed to handle state reply");
            return hardware_interface::return_type::ERROR;
        }
        curr_read_errors_ = 0;  // Reset read error count on success
    } else if (std::holds_alternative<serial_protocol::Error>(result)) {
        const auto& error = std::get<serial_protocol::Error>(result);
        if (increment_and_check_read_errors()) {
            RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"), "Protocol error: %s", error.message.c_str());
            return hardware_interface::return_type::ERROR;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("BrutusbotInterface"), "Protocol error: %s, retrying...",
                        error.message.c_str());
            return hardware_interface::return_type::OK;
        }
    } else {
        if (increment_and_check_read_errors()) {
            RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"), "Unexpected protocol result");
            return hardware_interface::return_type::ERROR;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("BrutusbotInterface"), "Unexpected protocol result, retrying...");
            return hardware_interface::return_type::OK;
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type BrutusbotInterface::write(const rclcpp::Time& /*time*/,
                                                          const rclcpp::Duration& /*period*/) {
    if (!serial_stream_.IsOpen()) {
        return hardware_interface::return_type::ERROR;
    }

    const std::size_t left_index = find_joint_index(kLeftJointName);
    const std::size_t right_index = find_joint_index(kRightJointName);
    if (left_index >= hw_commands_.size() || right_index >= hw_commands_.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"), "Could not locate left/right joints");
        return hardware_interface::return_type::ERROR;
    }

    auto const left_velocity_degrees = convert_radians_to_degrees(hw_commands_[left_index]);
    auto const right_velocity_degrees = convert_radians_to_degrees(hw_commands_[right_index]);

    // RCLCPP_INFO(rclcpp::get_logger("BrutusbotInterface"), "%.2f", hw_commands_[left_index]);

    const auto frame = serial_protocol_.encode_wheel_velocity_command(left_velocity_degrees, right_velocity_degrees);
    if (!send_frame(frame)) {
        RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"), "Failed to send set wheel velocity command frame");
        return hardware_interface::return_type::ERROR;
    }

    if (!wait_data_available(std::chrono::milliseconds(2))) {
        if (increment_and_check_write_errors()) {
            RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"),
                         "Timeout waiting for set velocity ok reply from serial port");
            return hardware_interface::return_type::ERROR;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("BrutusbotInterface"),
                        "Timeout waiting for set velocity ok reply from serial port, retrying...");
            return hardware_interface::return_type::OK;
        }
    }

    auto const result = process_serial_bytes();
    if (std::holds_alternative<serial_protocol::SetVelocityOk>(result)) {
        curr_write_errors_ = 0;  // Reset write error count on success
    } else if (std::holds_alternative<serial_protocol::Error>(result)) {
        const auto& error = std::get<serial_protocol::Error>(result);
        if (increment_and_check_write_errors()) {
            RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"), "Protocol error: %s", error.message.c_str());
            return hardware_interface::return_type::ERROR;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("BrutusbotInterface"), "Protocol error: %s, retrying...",
                        error.message.c_str());
            return hardware_interface::return_type::OK;
        }
    } else {
        if (increment_and_check_write_errors()) {
            RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"), "Unexpected protocol result");
            return hardware_interface::return_type::ERROR;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("BrutusbotInterface"), "Unexpected protocol result, retrying...");
            return hardware_interface::return_type::OK;
        }
    }

    return hardware_interface::return_type::OK;
}

bool BrutusbotInterface::validate_joint_names() const {
    if (info_.joints.size() != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"), "Expected exactly 2 joints, found %zu",
                     info_.joints.size());
        return false;
    }

    for (const auto& joint : info_.joints) {
        if (joint.name == kLeftJointName || joint.name == kRightJointName) {
            continue;
        }

        RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"), "Unexpected joint '%s' found", joint.name.c_str());
        return false;
    }

    return true;
}

void BrutusbotInterface::configure_serial_port() {
    auto const serial_port_param = info_.hardware_parameters.find("serial_port");
    if (serial_port_param != info_.hardware_parameters.end()) {
        serial_port_ = serial_port_param->second;
    } else {
        serial_port_ = "/dev/ttyUSB0";  // Default serial port
    }
}

bool BrutusbotInterface::open_serial_port() {
    if (serial_stream_.IsOpen()) {
        return true;
    }

    try {
        serial_stream_.Open(serial_port_);
        serial_stream_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        serial_stream_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_stream_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_stream_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_stream_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        serial_stream_.FlushIOBuffers();
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"), "Failed to open serial port '%s': %s",
                     serial_port_.c_str(), ex.what());
        return false;
    }

    if (!serial_stream_.IsOpen()) {
        RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"), "Failed to open serial port '%s'", serial_port_.c_str());
        return false;
    }

    return true;
}

void BrutusbotInterface::close_serial_port() {
    if (serial_stream_.IsOpen()) {
        serial_stream_.Close();
    }
}

bool BrutusbotInterface::read_initial_state() {
    for (int i = 0; i < 10; ++i) {
        const auto state_request = serial_protocol_.encode_state_request();
        if (!send_frame(state_request)) {
            RCLCPP_WARN(rclcpp::get_logger("BrutusbotInterface"), "Failed to send state request frame, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if (!wait_data_available(std::chrono::milliseconds(5))) {
            RCLCPP_WARN(rclcpp::get_logger("BrutusbotInterface"), "Timeout waiting for state reply, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        auto const result = process_serial_bytes();
        if (std::holds_alternative<serial_protocol::StateReply>(result)) {
            RCLCPP_INFO(rclcpp::get_logger("BrutusbotInterface"), "ACTIVATE SUCCESS - Received state reply");
            return true;
        } else if (std::holds_alternative<serial_protocol::Error>(result)) {
            const auto& error = std::get<serial_protocol::Error>(result);
            RCLCPP_WARN(rclcpp::get_logger("BrutusbotInterface"), "Protocol error: %s, retrying...",
                        error.message.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("BrutusbotInterface"), "Unexpected protocol result, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
    }

    RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"), "Failed to receive state reply after multiple attempts");
    return false;
}

bool BrutusbotInterface::send_frame(const std::vector<uint8_t>& frame) {
    if (frame.empty()) {
        return false;
    }

    try {
        serial_stream_.write(reinterpret_cast<const char*>(frame.data()), static_cast<std::streamsize>(frame.size()));
        serial_stream_.flush();
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("BrutusbotInterface"), "Failed to send serial frame: %s", ex.what());
        return false;
    }

    return true;
}

bool BrutusbotInterface::wait_data_available(std::chrono::milliseconds timeout) {
    auto start_time = std::chrono::steady_clock::now();
    while (!serial_stream_.IsDataAvailable()) {
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            return false;  // Timeout
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));  // Sleep briefly to avoid busy waiting
    }
    return true;  // Data is available
}

serial_protocol::FrameResult BrutusbotInterface::process_serial_bytes() {
    while (serial_stream_.IsDataAvailable()) {
        char byte = 0;
        serial_stream_.get(byte);
        serial_protocol::Frame frame;
        if (serial_protocol_.parse_byte(static_cast<uint8_t>(byte), &frame)) {
            return serial_protocol_.handle_frame(frame);
        }
    }
    return serial_protocol::FrameResult{serial_protocol::Error{"No valid frame received"}};
}

bool BrutusbotInterface::handle_state_reply(const serial_protocol::StateReply& state_reply) {
    const std::size_t left_index = find_joint_index(kLeftJointName);
    const std::size_t right_index = find_joint_index(kRightJointName);
    if (left_index >= hw_states_position_.size() || right_index >= hw_states_position_.size()) {
        return false;
    }

    hw_states_position_[left_index] = convert_degrees_to_radians(state_reply.left_position);
    hw_states_velocity_[left_index] = convert_degrees_to_radians(state_reply.left_velocity);
    hw_states_position_[right_index] = convert_degrees_to_radians(state_reply.right_position);
    hw_states_velocity_[right_index] = convert_degrees_to_radians(state_reply.right_velocity);
    return true;
}

std::size_t BrutusbotInterface::find_joint_index(const std::string& joint_name) const {
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        if (info_.joints[i].name == joint_name) {
            return i;
        }
    }

    return std::numeric_limits<std::size_t>::max();
}

float BrutusbotInterface::convert_radians_to_degrees(double radians) const {
    return static_cast<float>(radians * 180.0 / M_PI);
}

double BrutusbotInterface::convert_degrees_to_radians(float degrees) const {
    return static_cast<double>(degrees * M_PI / 180.0);
}

bool BrutusbotInterface::increment_and_check_read_errors() { return ++curr_read_errors_ >= kMaxReadErrors; }

bool BrutusbotInterface::increment_and_check_write_errors() { return ++curr_write_errors_ >= kMaxWriteErrors; }

}  // namespace brutusbot_firmware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(brutusbot_firmware::BrutusbotInterface, hardware_interface::SystemInterface)
