#include <SimpleFOC.h>

// For SameSky AMT10E3 - Parameters: Pin A, Pin B, CPR (PPR x 4), Pin X
Encoder encoder = Encoder(2, 3, 5120, 1);

void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }
void doI() { encoder.handleIndex(); }

void setup() {
    Serial.begin(115200);
    encoder.init();
    encoder.enableInterrupts(doA, doB, doI);
    Serial.println("AMT10E3 Encoder Test");
    Serial.println("---");
}

void loop() {
    encoder.update();

    float angle = encoder.getAngle();
    float degrees = angle * (180.0 / PI);
    float velocity = encoder.getVelocity();

    Serial.print("degrees:");
    Serial.print(degrees);
    Serial.print("\t");
    Serial.print("velocity [rad/s]:");
    Serial.println(velocity);
}