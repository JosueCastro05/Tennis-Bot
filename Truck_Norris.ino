#include <Bluepad32.h>

// Define motor control pins
const int M1A = 12;  // Left motor pin A
const int M1B = 14; // Left motor pin B
const int M2A = 2; // Right motor pin A
const int M2B = 4; // Right motor pin B

// Roller Mechanism pin
const int M1A_2 = 18;
const int M2B_2 = 33;

// Defining variables for the initial time for button / when button is held / and how long it must be held
unsigned long lastPressTimes[8] = {0};

bool buttonHelds[8] = {false};
const unsigned long holdDuration = 300; // 1000 is one second


ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
// test comment
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}



// Define motor control functions for turning, accelerating, and braking
void turnLeft() {
    // Adjust motor speeds to turn left (e.g., stop right motor or reduce speed)
    //analogWrite(leftMotorPin, 0); // Left motor stops
    //analogWrite(rightMotorPin, 255); // Right motor full speed
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, LOW);
    digitalWrite(M2A, HIGH);
    digitalWrite(M2B, LOW);
    
}

void turnRight() {
    // Adjust motor speeds to turn right (e.g., stop left motor or reduce speed)
    //analogWrite(leftMotorPin, 255); // Left motor full speed
    //analogWrite(rightMotorPin, 0);  // Right motor stops
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, HIGH);
    digitalWrite(M2A, LOW);
    digitalWrite(M2B, LOW);
  
}

void accelerate() {
    // Increase motor speed to move forward
    //analogWrite(leftMotorPin, 255);  // Full speed for left motor
    //analogWrite(rightMotorPin, 255); // Full speed for right motor
    digitalWrite(M1A, HIGH);
    digitalWrite(M1B, LOW);
    digitalWrite(M2A, LOW);
    digitalWrite(M2B, HIGH);
   
}

void brakeOrReverse() {
    // Reverse direction or brake (example: reversing by reversing motor polarity)
    //analogWrite(leftMotorPin, -255);  // Negative for reverse
    //analogWrite(rightMotorPin, -255); // Negative for reverse
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, HIGH);
    digitalWrite(M2A, HIGH);
    digitalWrite(M2B, LOW);
    
    
}

void stopMotors() {
    // Stop all motors
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, LOW);
    digitalWrite(M2A, LOW);
    digitalWrite(M2B, LOW);

}

void stopRoller() {
    // Stop roller

    digitalWrite(M1A_2, LOW);
    digitalWrite(M2B_2, LOW);

}

void rolling() {
    digitalWrite(M1A_2, HIGH);
    digitalWrite(M2B_2, HIGH);
}




void processGamepad(ControllerPtr ctl) {
    // Check if R1 (Right Shoulder button) is pressed to turn right
    if (ctl->buttons() == 0x0020) {
        // Code to turn right (e.g., control steering right)
        int btnIndex = 0;

        if (lastPressTimes[btnIndex] == 0) {
            lastPressTimes[btnIndex] = millis(); // Records tthe time when the button is first pressed
        }

        if (!buttonHelds[btnIndex] && (millis() - lastPressTimes[btnIndex] >= holdDuration)) {
            buttonHelds[btnIndex] = true;
            Serial.println("Turning right");
            turnRight();
        }
        
    } else {
        lastPressTimes[0] = 0;
        buttonHelds[0] = false;
    }

    // Check if L1 (Left Shoulder button) is pressed to turn left
    if (ctl->buttons() == 0x0010) {
        // Code to turn left (e.g., control steering left)
       int btnIndex = 1;

        if (lastPressTimes[btnIndex] == 0){
            lastPressTimes[btnIndex] = millis(); // Records the tume when the buttom is first pressed
        }

        if (!buttonHelds[btnIndex] && (millis() - lastPressTimes[btnIndex] >= holdDuration)) {
            buttonHelds[btnIndex] = true;
            Serial.println("Turning left");
            turnLeft();
        }
    
    } else {
        lastPressTimes[1] = 0;
        buttonHelds[1]= false;
    }

    // Check if Square (typically the 'X' button on PlayStation controllers) is pressed to brake or reverse
    if (ctl->buttons() == 0x0004) {
        // Code to brake or reverse
        int btnIndex = 2;

        if (lastPressTimes[btnIndex] == 0){
            lastPressTimes[btnIndex] = millis(); // Records the time when the button is first pressed
        }

        if (!buttonHelds[btnIndex] && (millis() - lastPressTimes[btnIndex] >= holdDuration)){
            buttonHelds[btnIndex] = true;
            Serial.println("Braking / Reversing");
            brakeOrReverse();
        } 
        
        // Add code here to brake or reverse in your system
      
    } else {
        lastPressTimes[2] = 0;
        buttonHelds[2] = false;
    }
        

    // Check if X (typically the 'A' button on PlayStation controllers) is pressed to accelerate
    if (ctl->buttons() == 0x0001) {
        // Code to accelerate
        int btnIndex = 3;
       

        if (lastPressTimes[btnIndex] == 0){
            lastPressTimes[btnIndex] = millis(); // Records the time when the button is first pressed
        }

        if (!buttonHelds[btnIndex] && (millis() - lastPressTimes[btnIndex] >= holdDuration)){
            buttonHelds[btnIndex] = true;
            Serial.println("Accelerating");
            accelerate();
        } 
        

    } else {
        lastPressTimes[3] = 0;
        buttonHelds[3] = false;
    }

    static bool prevRollingButtonState = false;
    static bool isRolling = false;

    bool currentRollingButtonState = ctl->buttons() & 0x0080;

    if (currentRollingButtonState && !prevRollingButtonState) {
        // Toggle the state
        isRolling = !isRolling;

        if (isRolling) {
            Serial.println("Rolling");
            rolling(); // start motor
        } else {
            Serial.println("Stop Rolling");
            stopRoller(); // stop motor
        }
    }

    // Update the previous state
    prevRollingButtonState = currentRollingButtonState;

    if (!isRolling) {
        Serial.println("Stop roller");
        stopRoller();
    }

   


    
    if (ctl->buttons() == 0x0000) {
        Serial.println("Stopped");
        stopMotors();
    } 


}



void dumpGamepad(ControllerPtr ctl) {
    // Serial.printf(
    //     "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    //     "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    //     ctl->index(),        // Controller Index
    //     ctl->dpad(),         // D-pad
    //     ctl->buttons(),      // bitmask of pressed buttons
    //     ctl->axisX(),        // (-511 - 512) left X Axis
    //     ctl->axisY(),        // (-511 - 512) left Y axis
    //     ctl->axisRX(),       // (-511 - 512) right X axis
    //     ctl->axisRY(),       // (-511 - 512) right Y axis
    //     ctl->brake(),        // (0 - 1023): brake button
    //     ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    //     ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    //     ctl->gyroX(),        // Gyro X
    //     ctl->gyroY(),        // Gyro Y
    //     ctl->gyroZ(),        // Gyro Z
    //     ctl->accelX(),       // Accelerometer X
    //     ctl->accelY(),       // Accelerometer Y
    //     ctl->accelZ()        // Accelerometer Z
    // );
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, "
        "misc: 0x%02x\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->miscButtons()  // bitmask of pressed "misc" buttons
    );
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected()) {
          dumpGamepad(myController);  // Ensure data is showing
          processGamepad(myController);
        }
    }
}




// Arduino setup function. Runs in CPU 1
void setup() {

    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);
    // Add these lines to configure motor pins
    pinMode(M1A, OUTPUT);
    pinMode(M1B, OUTPUT);
    pinMode(M2A, OUTPUT);
    pinMode(M2B, OUTPUT);
    pinMode(M1A_2, OUTPUT);
    pinMode(M2B_2, OUTPUT);
   
    
    // Initialize motors to stopped state
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, LOW);
    digitalWrite(M2A, LOW);
    digitalWrite(M2B, LOW);
    digitalWrite(M1A_2, LOW);
    digitalWrite(M2B_2, LOW);
    
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();
        


    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    delay(150);
}

