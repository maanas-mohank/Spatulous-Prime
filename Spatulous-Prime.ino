#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "0f045bbb-6f38-46b0-a13a-97bb5bcf9e7e"
#define CHARACTERISTIC_UUID "7f4b4a04-60ea-4afc-9289-d7805c914002"

// Motor control pins
const int motorA_IN1 = 23;
const int motorA_IN2 = 22;
const int motorB_IN3 = 21;
const int motorB_IN4 = 19;

// Ultrasonic sensor pins
const int TRIG_PIN = 17;
const int ECHO_PIN = 16;

// Obstacle detection threshold in cm
const int OBSTACLE_DISTANCE_CM = 20;

// Timing for avoidance arc (in milliseconds)
const int TURN_TIME    = 400;
const int FORWARD_TIME = 600;

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
char lastCommand = 's';

// ============================================================
//  SERIAL MONITOR HELPER
// ============================================================
void printSeparator() {
    Serial.println("--------------------------------------------");
}

void printHeader(const char* title) {
    printSeparator();
    Serial.print("  >> ");
    Serial.println(title);
    printSeparator();
}

void printStatus(const char* label, const char* value) {
    Serial.print("  [");
    Serial.print(label);
    Serial.print("] ");
    Serial.println(value);
}

void printStatusInt(const char* label, long value, const char* unit) {
    Serial.print("  [");
    Serial.print(label);
    Serial.print("] ");
    Serial.print(value);
    Serial.println(unit);
}

// ============================================================
//  ULTRASONIC SENSOR
// ============================================================
long getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration == 0) {
        Serial.println("  [SENSOR] No echo received — path clear (timeout)");
        return 999;
    }

    long distance = duration * 0.034 / 2;
    printStatusInt("SENSOR", distance, " cm");
    return distance;
}

// ============================================================
//  RAW MOTOR HELPERS (no obstacle check)
// ============================================================
void rawForward() {
    digitalWrite(motorA_IN1, HIGH);
    digitalWrite(motorA_IN2, LOW);
    digitalWrite(motorB_IN3, HIGH);
    digitalWrite(motorB_IN4, LOW);
    Serial.println("Moving Forward");
}

void rawTurnRight() {
    digitalWrite(motorA_IN1, HIGH);
    digitalWrite(motorA_IN2, LOW);
    digitalWrite(motorB_IN3, LOW);
    digitalWrite(motorB_IN4, LOW);
}

void rawTurnLeft() {
    digitalWrite(motorA_IN1, LOW);
    digitalWrite(motorA_IN2, LOW);
    digitalWrite(motorB_IN3, HIGH);
    digitalWrite(motorB_IN4, LOW);
}

void rawStop() {
    digitalWrite(motorA_IN1, LOW);
    digitalWrite(motorA_IN2, LOW);
    digitalWrite(motorB_IN3, LOW);
    digitalWrite(motorB_IN4, LOW);
}

// ============================================================
//  OBSTACLE AVOIDANCE
// ============================================================
void avoidObstacle() {
    printHeader("OBSTACLE AVOIDANCE TRIGGERED");
    long dist = getDistance();
    printStatusInt("Distance at trigger", dist, " cm");

    Serial.println("  [AVOID] Step 1: Turning RIGHT...");
    rawTurnRight();
    delay(TURN_TIME);

    Serial.println("  [AVOID] Step 2: Moving FORWARD (arc)...");
    rawForward();
    delay(FORWARD_TIME);

    Serial.println("  [AVOID] Step 3: Turning LEFT (realign)...");
    rawTurnLeft();
    delay(TURN_TIME);

    Serial.println("  [AVOID] Avoidance complete.");
    printStatusInt("Resuming command", lastCommand, "");
    printSeparator();

    controlMotors(lastCommand);
}

// ============================================================
//  MOTOR CONTROL
// ============================================================
void controlMotors(char command) {
    switch (command) {
        case 'f':
            Serial.println("  [MOTOR] Command: FORWARD");
            {
                long dist = getDistance();
                if (dist <= OBSTACLE_DISTANCE_CM) {
                    printStatusInt("Obstacle detected at", dist, " cm — avoiding");
                    avoidObstacle();
                } else {
                    printStatusInt("Path clear at", dist, " cm — moving forward");
                    rawForward();
                }
            }
            break;

        case 'b':
            Serial.println("  [MOTOR] Command: BACKWARD");
            rawForward(); // brief stop before reversing (optional safety)
            delay(50);
            digitalWrite(motorA_IN1, LOW);
            digitalWrite(motorA_IN2, HIGH);
            digitalWrite(motorB_IN3, LOW);
            digitalWrite(motorB_IN4, HIGH);
            break;

        case 'l':
            Serial.println("  [MOTOR] Command: TURN LEFT");
            rawTurnLeft();
            break;

        case 'r':
            Serial.println("  [MOTOR] Command: TURN RIGHT");
            rawTurnRight();
            break;

        case 's':
            Serial.println("  [MOTOR] Command: STOP");
            rawStop();
            break;

        default:
            Serial.print("  [MOTOR] Unknown command received: ");
            Serial.println(command);
            break;
    }
}

// ============================================================
//  BLE SERVER CALLBACKS
// ============================================================
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
        deviceConnected = true;
        printHeader("BLE CLIENT CONNECTED");
        printStatus("Status", "Device connected and ready");
        printSeparator();
    }

    void onDisconnect(BLEServer *pServer) {
        deviceConnected = false;
        rawStop();
        printHeader("BLE CLIENT DISCONNECTED");
        printStatus("Motors", "Stopped for safety");
        printStatus("BLE", "Restarting advertising...");
        printSeparator();
        BLEDevice::getAdvertising()->start();
    }
};

// ============================================================
//  BLE CHARACTERISTIC CALLBACKS
// ============================================================
class MyCharCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        String value = pChar->getValue().c_str();
        if (value.length() > 0) {
            char command = value[0];
            printSeparator();
            Serial.print("  [BLE] New command received: '");
            Serial.print(command);
            Serial.println("'");
            lastCommand = command;
            controlMotors(command);
        }
    }
};

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500); // Let serial settle

    printHeader("ESP32 BLE MOTOR CONTROLLER BOOTING");

    // Motor pins
    Serial.println("  [INIT] Setting up motor pins...");
    pinMode(motorA_IN1, OUTPUT);
    pinMode(motorA_IN2, OUTPUT);
    pinMode(motorB_IN3, OUTPUT);
    pinMode(motorB_IN4, OUTPUT);
    printStatus("Motor pins", "OK");

    // Ultrasonic pins
    Serial.println("  [INIT] Setting up ultrasonic sensor...");
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    printStatus("Ultrasonic pins", "OK");
    printStatusInt("Obstacle threshold", OBSTACLE_DISTANCE_CM, " cm");

    // Stop motors initially
    rawStop();
    printStatus("Motors", "Stopped (safe state)");

    // BLE init
    Serial.println("  [INIT] Initializing BLE...");
    BLEDevice::init("ESP32_Motor_Controller");
    printStatus("BLE Device", "ESP32_Motor_Controller");

    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE
    );
    pCharacteristic->setCallbacks(new MyCharCallbacks());
    pCharacteristic->setValue("s");

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    printStatus("BLE", "Advertising started");
    printHeader("READY — Waiting for BLE connection...");
}

// ============================================================
//  LOOP — Continuous forward obstacle check
// ============================================================
void loop() {
    rawForward();
    // if (deviceConnected && lastCommand == 'f') {
    //     long dist = getDistance();
    //     if (dist <= OBSTACLE_DISTANCE_CM) {
    //         Serial.println("  [LOOP] Obstacle detected while moving forward!");
    //         avoidObstacle();
    //     }
    // }
    // delay(100);
}