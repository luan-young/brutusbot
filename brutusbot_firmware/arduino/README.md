# Arduino protocol example

This folder contains a minimal Arduino sketch that speaks the same framed serial protocol used by the ROS 2 interface.

## Protocol summary

- Start byte: `0x7E`
- Payload size byte
- Simple checksum of the payload size
- Payload: first byte is the message code, remaining bytes are data
- Final checksum over the payload size and payload contents

## Supported codes

- `0x10`: wheel velocity command
- `0x20`: request state
- `0x21`: state reply

## Usage

1. Open [brutusbot_protocol.ino](brutusbot_protocol.ino) in the Arduino IDE.
2. Upload it to an Arduino Nano-compatible board.
3. Connect the board to the host over serial at 115200 baud.
4. The ROS 2 side will send wheel velocity commands and request state replies through the same framing.
