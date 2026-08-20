#ifndef BRUTUSBOT_FIRMWARE_BRUTUSBOT_INTERFACE_HPP_
#define BRUTUSBOT_FIRMWARE_BRUTUSBOT_INTERFACE_HPP_

#include <libserial/SerialStream.h>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <vector>

#include "brutusbot_firmware/serial_protocol.hpp"

namespace brutusbot_firmware {

constexpr int kMaxReadErrors = 3;
constexpr int kMaxWriteErrors = 3;

class BrutusbotInterface : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    bool validate_joint_names() const;
    void configure_serial_port();
    bool open_serial_port();
    void close_serial_port();
    bool read_initial_state();
    bool send_frame(const std::vector<uint8_t>& frame);
    bool wait_data_available(std::chrono::milliseconds timeout);
    serial_protocol::FrameResult process_serial_bytes();
    bool handle_state_reply(const serial_protocol::StateReply& state_reply);
    std::size_t find_joint_index(const std::string& joint_name) const;
    float convert_radians_to_degrees(double radians) const;
    double convert_degrees_to_radians(float degrees) const;
    bool increment_and_check_read_errors();
    bool increment_and_check_write_errors();

    std::string serial_port_;
    LibSerial::SerialStream serial_stream_;
    serial_protocol::FramedProtocol serial_protocol_;
    int curr_read_errors_ = 0;
    int curr_write_errors_ = 0;

    std::vector<double> hw_commands_;
    std::vector<double> hw_states_position_;
    std::vector<double> hw_states_velocity_;
};
}  // namespace brutusbot_firmware

#endif  // BRUTUSBOT_FIRMWARE_BRUTUSBOT_INTERFACE_HPP_
