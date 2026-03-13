#include <SimpleFOC.h>

// ===== ENCODER =====
// SameSky AMT10E3: PPR=1280, CPR=5120 (x4 quadrature)
// Index pin omitted — causes initFOC index search failure on this hardware.
Encoder encoder = Encoder(2, 3, 5120);  // A, B, CPR  (no index)
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

// ===== MOTOR & DRIVER =====
// GM4108: 22 poles = 11 pole pairs
BLDCMotor motor       = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 8, 7, 6);  // UH, VH, WH, EN

// ===== GEAR RATIO =====
// Measured during homing. Falls back to 6.0 if homing skipped.
float gearRatio      = 6.0;
bool  gearRatioKnown = false;

// ===== HOMING =====
const float    HOMING_VELOCITY  = 20.0f;  // [rad/s motor shaft]
const float    HOMING_VOLTAGE   = 6.0f;   // [V] needs headroom above the ~3V movement threshold
const uint32_t HOMING_DRIVE_MS  = 2000;   // [ms] drive time each direction

enum HomingState {
    HOMING_IDLE,
    HOMING_DRIVE_TO_STOP1,  // auto: drive negative for HOMING_DRIVE_MS
    HOMING_DRIVE_TO_STOP2,  // auto: drive positive for HOMING_DRIVE_MS
    HOMING_MANUAL_STOP1,    // manual: motor off, user moves to stop1, sends S
    HOMING_MANUAL_STOP2,    // manual: motor off, user moves to stop2, sends S
    HOMING_DONE
};
HomingState homingState  = HOMING_IDLE;
uint32_t    homingStartMs = 0;
bool        isHomed       = false;

float homeMotorAngle  = 0.0;
float stop1MotorAngle = 0.0;
float stop2MotorAngle = 0.0;

// ===== TARGET =====
// Output shaft degrees relative to home. Clamped 0–180 after homing.
float outputTargetDeg = 0.0;

// ===== HELPERS =====
inline float deg2rad(float d) { return d * (PI / 180.0f); }
inline float rad2deg(float r) { return r * (180.0f / PI); }
inline float outputDegToMotorRad(float deg) {
    return homeMotorAngle + deg2rad(deg) * gearRatio;
}

// ===== PROTOTYPES =====
void startAutoHoming();
void startManualHoming();
void commitHome();
void enterAngleMode();

// ===== COMMANDER =====
Commander command = Commander(Serial);
void doTarget(char* cmd)       { command.scalar(&outputTargetDeg, cmd); }
void doLimit(char* cmd)        { command.scalar(&motor.voltage_limit, cmd); }
void doHomingAuto(char* cmd)   { startAutoHoming(); }
void doHomingManual(char* cmd) { startManualHoming(); }
void doSetHome(char* cmd)      { commitHome(); }
void doPosition(char* cmd) {
    float motorAngle = motor.shaft_angle;

    if (!isHomed) {
        Serial.print("[POS] Motor shaft (raw) : ");
        Serial.print(rad2deg(motorAngle), 2);
        Serial.println(" deg  (not homed)");
        return;
    }

    // Invert outputDegToMotorRad: motorAngle = homeMotorAngle + deg2rad(deg)*gearRatio
    // => outputDeg = rad2deg((motorAngle - homeMotorAngle) / gearRatio)
    // This is guaranteed to match what the loop commands.
    float outputDeg     = rad2deg((motorAngle - homeMotorAngle) / gearRatio);
    float effectiveTarget = gearRatioKnown
        ? constrain(outputTargetDeg, 0.0f, 180.0f)
        : outputTargetDeg;

    Serial.print("[POS] Position : "); Serial.print(outputDeg, 2);        Serial.println(" deg");
    Serial.print("[POS] Target   : "); Serial.print(effectiveTarget, 2);  Serial.println(" deg");
    Serial.print("[POS] Error    : "); Serial.print(outputDeg - effectiveTarget, 2); Serial.println(" deg");
    Serial.print("[POS] Motor shaft : "); Serial.print(rad2deg(motorAngle), 2); Serial.println(" deg (raw)");
    Serial.print("[POS] Home  shaft : "); Serial.print(rad2deg(homeMotorAngle), 2); Serial.println(" deg (raw)");
    Serial.print("[POS] Gear ratio  : "); Serial.println(gearRatio, 4);
}

// ================================================================
void setup() {
    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);

    encoder.init();
    encoder.enableInterrupts(doA, doB);

    driver.pwm_frequency        = 32000;
    driver.voltage_power_supply = 20;
    driver.voltage_limit        = 20;
    if (!driver.init()) { Serial.println("[ERROR] Driver init failed!"); while (1); }

    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);

    motor.torque_controller = TorqueControlType::voltage;
    motor.controller        = MotionControlType::angle;
    motor.updateVoltageLimit(5.0);
    motor.updateVelocityLimit(30.0);

    motor.PID_velocity.P           = 0.15;
    motor.PID_velocity.I           = 5.0;
    motor.PID_velocity.D           = 0.0;
    motor.PID_velocity.output_ramp = 1000.0;
    motor.P_angle.P                = 3.0;   // low — gearbox inertia amplifies overshoot
    motor.LPF_velocity.Tf          = 0.05;  // smoother velocity feedback reduces hunting

    if (!motor.init()) { Serial.println("[ERROR] Motor init failed!"); while (1); }

    motor.useMonitoring(Serial);
    Serial.println("Running FOC calibration (motor will wiggle briefly)...");
    motor.initFOC();
    // After first run hardcode: motor.initFOC(1.23f, Direction::CW);

    if (!motor.enabled) { Serial.println("[ERROR] initFOC failed!"); while (1); }

    // Default home at boot — T commands work immediately
    homeMotorAngle  = motor.shaft_angle;
    outputTargetDeg = 0.0;
    isHomed         = true;

    command.add('T', doTarget,       "target output angle [deg]");
    command.add('L', doLimit,        "voltage limit [V]");
    command.add('H', doHomingAuto,   "auto-home: drive each way, measure gear ratio");
    command.add('M', doHomingManual, "manual home: move to stop1 by hand, then S, then stop2, then S");
    command.add('S', doSetHome,      "confirm each manual homing step");
    command.add('P', doPosition,     "print current output shaft position [deg]");

    Serial.println("Ready! T0 = current position.");
    Serial.println("  H      = auto-home");
    Serial.println("  M -> S -> S = manual home (stop1 then stop2)");
    Serial.println("  T<deg> = move    L<V> = voltage    P = position");
}

// ================================================================
void loop() {
    motor.loopFOC();

    switch (homingState) {

        case HOMING_DRIVE_TO_STOP1:
            motor.move(-HOMING_VELOCITY);
            if (millis() - homingStartMs >= HOMING_DRIVE_MS) {
                stop1MotorAngle = motor.shaft_angle;
                Serial.print("[HOMING] Stop 1 recorded. Motor angle = ");
                Serial.print(rad2deg(stop1MotorAngle), 1);
                Serial.println(" deg");
                Serial.println("[HOMING] Step 2/2: driving positive for 2s...");
                motor.PID_velocity.reset();
                homingStartMs = millis();
                homingState   = HOMING_DRIVE_TO_STOP2;
            }
            break;

        case HOMING_DRIVE_TO_STOP2:
            motor.move(+HOMING_VELOCITY);
            if (millis() - homingStartMs >= HOMING_DRIVE_MS) {
                stop2MotorAngle = motor.shaft_angle;
                Serial.print("[HOMING] Stop 2 recorded. Motor angle = ");
                Serial.print(rad2deg(stop2MotorAngle), 1);
                Serial.println(" deg");

                float motorTravelRad = stop2MotorAngle - stop1MotorAngle;  // signed
                gearRatio      = motorTravelRad / PI;
                gearRatioKnown = true;

                Serial.println("[HOMING] ================================");
                Serial.print  ("[HOMING] Motor shaft travel  : ");
                Serial.print  (rad2deg(fabsf(motorTravelRad)), 2); Serial.println(" deg");
                Serial.println("[HOMING] Output shaft travel : 180.00 deg");
                Serial.print  ("[HOMING] Measured gear ratio : ");
                Serial.println(fabsf(gearRatio), 4);
                Serial.println("[HOMING] ================================");

                // Sanity check — if motor barely moved, homing failed
                if (fabsf(gearRatio) < 0.5f) {
                    Serial.println("[ERROR] Gear ratio too small — motor did not move during homing!");
                    Serial.println("        Check HOMING_VOLTAGE is above the movement threshold.");
                    Serial.println("        Re-run H to try again.");
                    enterAngleMode();
                    homingState = HOMING_IDLE;
                    return;
                }
                homingState = HOMING_IDLE;
                commitHome();
            }
            break;

        case HOMING_MANUAL_STOP1:
        case HOMING_MANUAL_STOP2:
            // Motor disabled — waiting for S command
            break;

        case HOMING_IDLE:
        case HOMING_DONE:
        default: {
            float clampedDeg = gearRatioKnown
                ? constrain(outputTargetDeg, 0.0f, 180.0f)
                : outputTargetDeg;
            motor.move(outputDegToMotorRad(clampedDeg));
            break;
        }
    }

    command.run();
}

// ================================================================
void enterAngleMode() {
    motor.PID_velocity.reset();
    motor.P_angle.reset();
    motor.controller = MotionControlType::angle;
    motor.updateVoltageLimit(5.0);
    motor.move(motor.shaft_angle);  // seed setpoint at current position
}

void startAutoHoming() {
    motor.PID_velocity.reset();
    motor.controller = MotionControlType::velocity;
    motor.updateVoltageLimit(HOMING_VOLTAGE);

    Serial.println("[HOMING] Step 1/2: driving negative for 2s...");
    homingStartMs = millis();
    homingState   = HOMING_DRIVE_TO_STOP1;
}

void startManualHoming() {
    Serial.println("[HOMING] Manual step 1/2: motor DISABLED.");
    Serial.println("         Move output shaft to STOP 1 (0 deg end), then send S.");
    motor.disable();
    homingState = HOMING_MANUAL_STOP1;
}

void commitHome() {
    if (homingState == HOMING_MANUAL_STOP1) {
        // First S — record stop1, ask for stop2
        stop1MotorAngle = motor.shaft_angle;
        Serial.print("[HOMING] Stop 1 recorded. Motor angle = ");
        Serial.print(rad2deg(stop1MotorAngle), 1);
        Serial.println(" deg");
        Serial.println("[HOMING] Manual step 2/2: move output shaft to STOP 2 (180 deg end), then send S.");
        homingState = HOMING_MANUAL_STOP2;
        return;  // not finished yet
    }

    if (homingState == HOMING_MANUAL_STOP2) {
        // Second S — record stop2, compute gear ratio
        stop2MotorAngle = motor.shaft_angle;
        Serial.print("[HOMING] Stop 2 recorded. Motor angle = ");
        Serial.print(rad2deg(stop2MotorAngle), 1);
        Serial.println(" deg");

        float motorTravelRad = stop2MotorAngle - stop1MotorAngle;  // signed: negative if stop2 < stop1
        gearRatio      = motorTravelRad / PI;
        gearRatioKnown = true;

        Serial.println("[HOMING] ================================");
        Serial.print  ("[HOMING] Motor shaft travel  : ");
        Serial.print  (rad2deg(fabsf(motorTravelRad)), 2); Serial.println(" deg");
        Serial.println("[HOMING] Output shaft travel : 180.00 deg");
        Serial.print  ("[HOMING] Measured gear ratio : ");
        Serial.println(fabsf(gearRatio), 4);
        Serial.println("[HOMING] ================================");

        if (fabsf(gearRatio) < 0.5f) {
            Serial.println("[ERROR] Gear ratio too small — stops are too close together!");
            Serial.println("        Re-run M and place stops further apart.");
            motor.enable();
            enterAngleMode();
            homingState = HOMING_IDLE;
            return;
        }
        homeMotorAngle = stop1MotorAngle;
        enterAngleMode();
        motor.enable();
        Serial.println("[HOMING] Motor re-enabled.");
    } else {
        // Auto homing path — home = stop1
        homeMotorAngle = stop1MotorAngle;
        enterAngleMode();
    }

    // Drive to midpoint after homing
    outputTargetDeg = 90.0;
    isHomed         = true;
    homingState     = HOMING_DONE;

    Serial.print("[HOMING] Done. Gear ratio = ");
    Serial.print(fabsf(gearRatio), 3);
    Serial.println(". Driving to 90 deg (centre)...");
    Serial.println("         T0 = stop1,  T90 = centre,  T180 = stop2.");
}
