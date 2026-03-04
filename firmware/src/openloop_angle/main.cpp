#include <SimpleFOC.h>

// For SameSky AMT10E3 - Parameters: Pin A, Pin B, CPR (PPR x 4), Pin X
Encoder encoder = Encoder(2, 3, 5120, 1);

void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }
void doI() { encoder.handleIndex(); }

// iFlight GM4108 is 22P, so 11 pole pairs
BLDCMotor motor = BLDCMotor(11);

BLDCDriver3PWM driver = BLDCDriver3PWM(10, 8, 7, 6);

float target = 0;

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target, cmd); }
void doLimit(char* cmd)  { command.scalar(&motor.voltage_limit, cmd); }

void setup() {
    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);

    // Encoder
    encoder.init();
    encoder.enableInterrupts(doA, doB, doI);

    // Driver
    driver.voltage_power_supply = 12;
    driver.voltage_limit = 6;
    if (!driver.init()) {
        Serial.println("Driver init failed!");
        return;
    }
    motor.linkDriver(&driver);

    // Important: Start low, then increase until the motion is smooth
    // And keep current within motor specifications :)
    // Use V = √(P × R) to find safe limit
    motor.voltage_limit  = 3;
    motor.velocity_limit = 5;
    motor.controller = MotionControlType::angle_openloop;

    if (!motor.init()) {
        Serial.println("Motor init failed!");
        return;
    }

    command.add('T', doTarget, "target angle (rad)");
    command.add('L', doLimit,  "voltage limit");
    command.add('V', doLimitVelocity, "velocity limit")

    Serial.println("Motor ready!");
}

void loop() {
    motor.loopFOC();
    motor.move(target);
    command.run();
}