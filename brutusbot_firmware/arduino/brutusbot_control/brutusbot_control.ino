#include <Arduino.h>
#include <AccelStepper.h>

#include "serial_protocol.h"
#include "step_motor_definitions.h"

struct MotorState {
    float actPosition = 0.0f;
    float actVelocity = 0.0f;
    float ctrlVelocityCmd = 0.0f;
    float ctrlVelocityDemand = 0.0f;
};

ParserState parser;

// false used in last parameter to prevent enableOutputs() beeing automatically called and to allow pins invertions
AccelStepper motor1{AccelStepper::DRIVER, kMotor1_Pin_Clk, kMotor1_Pin_Dir, 0xFF, 0xFF, false};
AccelStepper motor2{AccelStepper::DRIVER, kMotor2_Pin_Clk, kMotor2_Pin_Dir, 0xFF, 0xFF, false};

bool ledState = false;
MotorState motorStateLeft;
MotorState motorStateRight;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial) {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
        delay(100);
    }

    ledState = false;
    digitalWrite(LED_BUILTIN, ledState);

    // invert the necessary pins with setPinsInverted (directionPin, stepPin, enablePin)
    // then configure the enable pin
    motor1.setPinsInverted(false, false, true);
    motor1.setEnablePin(kMotor1_Pin_Ena);
    motor1.disableOutputs();
    motor1.enableOutputs();

    motor2.setPinsInverted(false, false, true);
    motor2.setEnablePin(kMotor2_Pin_Ena);
    motor2.disableOutputs();
    motor2.enableOutputs();

    motor1.setMaxSpeed(kMotor1_MaxSpeed * kMotor1_FactorDegreesToEffSteps);
    motor1.setAcceleration(kMotor1_MaxAccel * kMotor1_FactorDegreesToEffSteps);

    motor2.setMaxSpeed(kMotor1_MaxSpeed * kMotor1_FactorDegreesToEffSteps);
    motor2.setAcceleration(kMotor1_MaxAccel * kMotor1_FactorDegreesToEffSteps);

    motor1.setSpeed(motorStateLeft.ctrlVelocityCmd);
    motor2.setSpeed(motorStateRight.ctrlVelocityCmd);
}

void loop() {
    while (Serial.available() > 0) {
        Frame frame;
        const uint8_t byte = static_cast<uint8_t>(Serial.read());
        if (parse_byte(byte, parser, frame)) {
            handle_frame(frame, motorStateLeft, motorStateRight);
        }
    }

    // No acceleration limitation, just set the speed cmd = speed demand
    motorStateLeft.ctrlVelocityCmd = motorStateLeft.ctrlVelocityDemand;
    motorStateRight.ctrlVelocityCmd = motorStateRight.ctrlVelocityDemand;

    motor1.setSpeed(motorStateLeft.ctrlVelocityCmd);
    motor2.setSpeed(motorStateRight.ctrlVelocityCmd);
    motor1.runSpeed();
    motor2.runSpeed();

    motorStateLeft.actPosition = motor1.currentPosition() * kMotor1_FactorEffStepsToDegrees;
    motorStateLeft.actVelocity = motor1.speed() * kMotor1_FactorEffStepsToDegrees;
    motorStateRight.actPosition = motor2.currentPosition() * kMotor1_FactorEffStepsToDegrees;
    motorStateRight.actVelocity = motor2.speed() * kMotor1_FactorEffStepsToDegrees;
}

void handle_frame(const Frame& frame, MotorState& motorStateLeft, MotorState& motorStateRight) {
    switch (frame.code) {
        case kSetWheelVelocity: {
            if (frame.data_size != 8) {
                return;
            }

            float left_velocity_cmd = 0.0f;
            float right_velocity_cmd = 0.0f;
            memcpy(&left_velocity_cmd, frame.data, sizeof(left_velocity_cmd));
            memcpy(&right_velocity_cmd, frame.data + sizeof(left_velocity_cmd), sizeof(right_velocity_cmd));

            motorStateLeft.ctrlVelocityDemand = left_velocity_cmd * kMotor1_FactorDegreesToEffSteps;
            motorStateRight.ctrlVelocityDemand = right_velocity_cmd * kMotor1_FactorDegreesToEffSteps;
            
            send_velocity_reply_ok();

            break;
        }

        case kRequestState:
            send_state_reply(motorStateLeft.actPosition, motorStateLeft.actVelocity, motorStateRight.actPosition,
                             motorStateRight.actVelocity);
            break;

        default:
            break;
    }
}
