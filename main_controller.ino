#include <Bluepad32.h>

// Motor control pins
const int M1A = 12;   // Left motor A
const int M1B = 14;   // Left motor B
const int M2A = 2;    // Right motor A
const int M2B = 4;    // Right motor B

// Roller mechanism pins
const int M1A_2 = 18;
const int M2B_2 = 33;

// Button press tracking
unsigned long lastPressTimes[8] = {0};
bool buttonHelds[8] = {false};
const unsigned long holdDuration = 300; // Hold threshold (ms)

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Callback for controller connection
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", 
                          ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, no available slot");
    }
}

// Callback for controller disconnection
void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected, index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }
    if (!foundController) {
        Serial.println("CALLBACK: Disconnected controller not found");
    }
}

// Motor control functions
void turnLeft() {
    digitalWrite(M1A, HIGH);
    digitalWrite(M1B, LOW);
    digitalWrite(M2A, HIGH);
    digitalWrite(M2B, LOW);
}

void turnRight() {
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, HIGH);
    digitalWrite(M2A, LOW);
    digitalWrite(M2B, HIGH);
}

void accelerate() {
    digitalWrite(M1A, HIGH);
    digitalWrite(M1B, LOW);
    digitalWrite(M2A, LOW);
    digitalWrite(M2B, HIGH);
}

void brakeOrReverse() {
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, HIGH);
    digitalWrite(M2A, HIGH);
    digitalWrite(M2B, LOW);
}

void stopMotors() {
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, LOW);
    digitalWrite(M2A, LOW);
    digitalWrite(M2B, LOW);
}

void stopRoller() {
    digitalWrite(M1A_2, LOW);
    digitalWrite(M2B_2, LOW);
}

void rolling() {
    digitalWrite(M1A_2, HIGH);
    digitalWrite(M2B_2, HIGH);
}

// Controller input processing
void processGamepad(ControllerPtr ctl) {
    // R1 → turn right
    if (ctl->buttons() == 0x0020) {
        int btnIndex = 0;
        if (lastPressTimes[btnIndex] == 0)
            lastPressTimes[btnIndex] = millis();
        if (!buttonHelds[btnIndex] && (millis() - lastPressTimes[btnIndex] >= holdDuration)) {
            buttonHelds[btnIndex] = true;
            Serial.println("Turning right");
            turnRight();
        }
    } else {
        lastPressTimes[0] = 0;
        buttonHelds[0] = false;
    }

    // L1 → turn left
    if (ctl->buttons() == 0x0010) {
        int btnIndex = 1;
        if (lastPressTimes[btnIndex] == 0)
            lastPressTimes[btnIndex] = millis();
        if (!buttonHelds[btnIndex] && (millis() - lastPressTimes[btnIndex] >= holdDuration)) {
            buttonHelds[btnIndex] = true;
            Serial.println("Turning left");
            turnLeft();
        }
    } else {
        lastPressTimes[1] = 0;
        buttonHelds[1] = false;
    }

    // Square → brake/reverse
    if (ctl->buttons() == 0x0004) {
        int btnIndex = 2;
        if (lastPressTimes[btnIndex] == 0)
            lastPressTimes[btnIndex] = millis();
        if (!buttonHelds[btnIndex] && (millis() - lastPressTimes[btnIndex] >= holdDuration)) {
            buttonHelds[btnIndex] = true;
            Serial.println("Braking / Reversing");
            brakeOrReverse();
        }
    } else {
        lastPressTimes[2] = 0;
        buttonHelds[2] = false;
    }

    // X → accelerate
    if (ctl->buttons() == 0x0001) {
        int btnIndex = 3;
        if (lastPressTimes[btnIndex] == 0)
            lastPressTimes[btnIndex] = millis();
        if (!buttonHelds[btnIndex] && (millis() - lastPressTimes[btnIndex] >= holdDuration)) {
            buttonHelds[btnIndex] = true;
            Serial.println("Accelerating");
            accelerate();
        }
    } else {
        lastPressTimes[3] = 0;
        buttonHelds[3] = false;
    }

    // Circle → toggle roller
    static bool prevRollingButtonState = false;
    static bool isRolling = false;

    bool currentRollingButtonState = ctl->buttons() & 0x0080;

    if (currentRollingButtonState && !prevRollingButtonState) {
        isRolling = !isRolling;
        if (isRolling) {
            Serial.println("Rolling");
            rolling();
        } else {
            Serial.println("Stop Rolling");
            stopRoller();
        }
    }

    prevRollingButtonState = currentRollingButtonState;

    if (!isRolling) {
        Serial.println("Stop roller");
        stopRoller();
    }

    // No input → stop motors
    if (ctl->buttons() == 0x0000) {
        Serial.println("Stopped");
        stopMotors();
    }
}

// Controller data logging
void dumpGamepad(ControllerPtr ctl) {
    Serial.printf("idx=%d, dpad: 0x%02x, buttons: 0x%04x, misc: 0x%02x\n",
                  ctl->index(), ctl->dpad(), ctl->buttons(), ctl->miscButtons());
}

// Controller polling
void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected()) {
            dumpGamepad(myController);
            processGamepad(myController);
        }
    }
}

// Setup
void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);

    pinMode(M1A, OUTPUT);
    pinMode(M1B, OUTPUT);
    pinMode(M2A, OUTPUT);
    pinMode(M2B, OUTPUT);
    pinMode(M1A_2, OUTPUT);
    pinMode(M2B_2, OUTPUT);

    // Initialize motors and roller off
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, LOW);
    digitalWrite(M2A, LOW);
    digitalWrite(M2B, LOW);
    digitalWrite(M1A_2, LOW);
    digitalWrite(M2B_2, LOW);
}

// Main loop
void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    delay(150);
}
