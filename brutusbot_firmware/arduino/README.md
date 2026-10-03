# Arduino motor controller

This folder contains the Arduino sketch that drives Brutusbot's two stepper motors and speaks the same framed serial
protocol used by the ROS 2 hardware interface (`brutusbot_firmware/BrutusbotInterface`).

## Files

- [brutusbot_control.ino](brutusbot_control/brutusbot_control.ino): main sketch, frame handling and motor control loop.
- [serial_protocol.h](brutusbot_control/serial_protocol.h): frame parser and encoder. Must stay in sync with
  `include/brutusbot_firmware/serial_protocol.hpp` on the ROS 2 side.
- [step_motor_definitions.h](brutusbot_control/step_motor_definitions.h): pin assignments, microstepping and
  speed/acceleration limits.

## Protocol summary

Serial at 115200 baud. Each frame is:

| Byte(s)  | Content                                                         |
|----------|-----------------------------------------------------------------|
| 1        | Start byte `0x7E`                                               |
| 1        | Payload size (message code + data, 1 to 33 bytes)              |
| 1        | Size checksum (equal to the payload size byte)                 |
| N        | Payload: first byte is the message code, remaining bytes data  |
| 1        | Checksum: 8-bit sum of the payload size byte and payload bytes |

Floats are sent as raw 32-bit little-endian values. Malformed or corrupted frames are dropped and the parser waits for
the next start byte.

## Supported codes

| Code   | Direction       | Data                                                                   |
|--------|-----------------|------------------------------------------------------------------------|
| `0x10` | host → Arduino  | Set wheel velocity: left, right velocity (2 floats, deg/s)            |
| `0x11` | host → Arduino  | Request state (no data)                                                |
| `0x20` | Arduino → host  | Velocity command acknowledged (no data)                                |
| `0x21` | Arduino → host  | State reply: left position, left velocity, right position, right velocity (4 floats, deg and deg/s) |

The ROS 2 interface converts between rad/s on the ROS side and degrees on the Arduino side.

## Usage

1. Install the [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/) library in the Arduino IDE.
2. Open [brutusbot_control.ino](brutusbot_control/brutusbot_control.ino) and upload it to the board.
3. Connect the board to the Raspberry Pi over USB serial. The hardware interface uses `/dev/ttyUSB0` by default (set
   by the `serial_port` parameter in `brutusbot_description/urdf/brutusbot_ros2_control.xacro`).
4. Check communication with `ros2 run brutusbot_firmware test_serial_protocol`, or start the full robot with
   `ros2 launch brutusbot_bringup real_robot.launch.py`.
