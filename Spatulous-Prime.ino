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

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// --- Motor Control Function ---
void controlMotors(char command) {
    switch (command) {
        case 'f': // Forward
            digitalWrite(motorA_IN1, HIGH);
            digitalWrite(motorA_IN2, LOW);
            digitalWrite(motorB_IN3, HIGH);
            digitalWrite(motorB_IN4, LOW);
            Serial.println("Moving Forward");
            break;
        case 'b': // Backward
            digitalWrite(motorA_IN1, LOW);
            digitalWrite(motorA_IN2, HIGH);
            digitalWrite(motorB_IN3, LOW);
            digitalWrite(motorB_IN4, HIGH);
            Serial.println("Moving Backward");
            break;
        case 'l': // Turn Left
            digitalWrite(motorA_IN1, LOW);
            digitalWrite(motorA_IN2, LOW);
            digitalWrite(motorB_IN3, HIGH);
            digitalWrite(motorB_IN4, LOW);
            Serial.println("Turning Left");
            break;
        case 'r': // Turn Right
            digitalWrite(motorA_IN1, HIGH);
            digitalWrite(motorA_IN2, LOW);
            digitalWrite(motorB_IN3, LOW);
            digitalWrite(motorB_IN4, LOW);
            Serial.println("Turning Right");
            break;
        case 's': // Stop
            digitalWrite(motorA_IN1, LOW);
            digitalWrite(motorA_IN2, LOW);
            digitalWrite(motorB_IN3, LOW);
            digitalWrite(motorB_IN4, LOW);
            Serial.println("Stopped");
            break;
        default:
            Serial.print("Unknown command: ");
            Serial.println(command);
            break;
    }
}

// --- Server Callbacks (handles connect/disconnect) ---
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
        deviceConnected = true;
        Serial.println("Client connected.");
    }

    void onDisconnect(BLEServer *pServer) {
        deviceConnected = false;
        Serial.println("Client disconnected. Restarting advertising...");
        BLEDevice::getAdvertising()->start(); // Restart advertising so new clients can connect
    }
};

// --- Characteristic Callbacks (handles incoming commands) ---
class MyCharCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        String value = pChar->getValue();
        if (value.length() > 0) {
            char command = value[0];
            Serial.print("Received command: ");
            Serial.println(command);
            controlMotors(command);
        }
    }
};

void setup() {
    Serial.begin(115200);

    // Set motor pins as outputs
    pinMode(motorA_IN1, OUTPUT);
    pinMode(motorA_IN2, OUTPUT);
    pinMode(motorB_IN3, OUTPUT);
    pinMode(motorB_IN4, OUTPUT);

    // Stop motors initially
    controlMotors('s');

    // Initialize BLE
    BLEDevice::init("ESP32_Motor_Controller");

    // Create BLE Server and attach server callbacks
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create BLE Characteristic with read/write properties
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE
    );

    // Attach characteristic callbacks
    pCharacteristic->setCallbacks(new MyCharCallbacks());

    // Set an initial value
    pCharacteristic->setValue("s");

    // Start the service
    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true); // Helps some phones discover the device
    pAdvertising->start();

    Serial.println("BLE Motor Controller ready. Waiting for connection...");
}

void loop() {
    // All motor logic is handled in MyCharCallbacks::onWrite()
    // Nothing needed here — keeping loop clean
    delay(10);
}
