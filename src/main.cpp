#include <Arduino.h>
#include <ArduinoJson.h>

// Deep Sleep
#define seconds_to_trigger_DeepSleep 600 // Timer in seconds until sleep mode is triggered 


// Timer
#define timer_prescale 64000
hw_timer_t *Timer0_Cfg = NULL;
int32_t counter = 0;
RTC_DATA_ATTR int bootCount = 0;

//BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID             "DFCD0001-36E1-4688-B7F5-EA07361B26A8"
#define START_CHARACTERISTIC_UUID "DFCD000A-36E1-4688-B7F5-EA07361B26A8"
#define STOP_CHARACTERISTIC_UUID  "DFCD000B-36E1-4688-B7F5-EA07361B26A8"
#define SENSOR_CHARACTERISTIC_UUID "DFCD000C-36E1-4688-B7F5-EA07361B26A8"

bool sendData = false; // Flag to indicate whether to send sensor data
bool resetDSTimer = false; // Flag to reset DeepSleep timer

BLEServer* pServer = NULL;
BLECharacteristic* pStartCharacteristic = NULL;
BLECharacteristic* pStopCharacteristic = NULL;
BLECharacteristic* pSensorCharacteristic=NULL;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        // On client connect
    }

    void onDisconnect(BLEServer* pServer) {
        // On client disconnect
        sendData = false; // Stop sending sensor data on disconnect
        BLEAdvertising* pAdvertising = pServer->getAdvertising();
        pAdvertising->start(); // Restart advertising
        Serial.println("Device disconnected, restarting advertising");
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();

        if (value.length() > 0) {
            if (pCharacteristic == pStartCharacteristic && value[0] == 0x01) {
                sendData = true; // Start sending sensor data
                resetDSTimer = true; //
            } else if (pCharacteristic == pStopCharacteristic && value[0] == 0x01) {
                sendData = false; // Stop sending sensor data
            }
        }
    }
};

void setupBLE() {
    BLEDevice::init("SmartEdge");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService* pService = pServer->createService(BLEUUID(SERVICE_UUID));

    pStartCharacteristic = pService->createCharacteristic(
                             BLEUUID(START_CHARACTERISTIC_UUID),
                             BLECharacteristic::PROPERTY_WRITE
                           );
    pStartCharacteristic->setCallbacks(new MyCallbacks());

    pStopCharacteristic = pService->createCharacteristic(
                             BLEUUID(STOP_CHARACTERISTIC_UUID),
                             BLECharacteristic::PROPERTY_WRITE
                           );
    pStopCharacteristic->setCallbacks(new MyCallbacks());

    // Create the characteristic for sensor data
    pSensorCharacteristic = pService->createCharacteristic(
                             BLEUUID(SENSOR_CHARACTERISTIC_UUID), // Replace with your sensor characteristic UUID
                             BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                           );

    pSensorCharacteristic->addDescriptor(new BLE2902());
    pService->start();

    //BLEAdvertising* pAdvertising = pServer->getAdvertising(); //this still is working for backward compatibility
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    
    BLEDevice::startAdvertising();
    //pAdvertising->start();
}

// GPIO Configuration
#define ledPIN 12
#define led1PIN 4
#define resetButtonPin 39 // GPIO 39
bool resetButtonPressed = false;

// Sensor Configuration
long lastMsg = 0;
bool bSensorValue = false; // flag for fetching sensor value
double sensorValue = 0;
String dataToSend;

#include "HX711.h"
//HX711 Configuration
HX711 scale;  // Initializes library functions.
double calibration_factor = -55080;//-20328; // Defines calibration factor we'll use for calibrating.
// HX711 circuit wiring
#define SDA 21
#define SCL 22

void IRAM_ATTR Timer0_ISR()
{
    counter++;
    esp_deep_sleep_start();
    
}


/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setup()
{   
    Serial.begin(9600);
    
    setupBLE();
    
    // Initialize Timer Interrupt
    Serial.println("Interrupt Initialized");
    Timer0_Cfg = timerBegin(0,timer_prescale, true); // Prescaler set to 80*1e6 which means timer ticks every second
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 1250*seconds_to_trigger_DeepSleep, true);
    timerAlarmEnable(Timer0_Cfg);

    //Increment boot number and print it every reboot
    ++bootCount;
    Serial.println("Boot number: " + String(bootCount));

    //Print the wakeup reason for ESP32
    print_wakeup_reason();

    /*
        First we configure the wake up source
        We set our ESP32 to wake up for an external trigger.
        There are two types for ESP32, ext0 and ext1 .
        ext0 uses RTC_IO to wakeup thus requires RTC peripherals
        to be on while ext1 uses RTC Controller so doesnt need
        peripherals to be powered on.
        Note that using internal pullups/pulldowns also requires
        RTC peripherals to be turned on.
    */
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_39, 1); //1 = High, 0 = Low


    /*
        Initialize Bluetooth Classic Interface
    */
   
    pinMode(ledPIN, OUTPUT);
    pinMode(led1PIN, OUTPUT);
    pinMode(resetButtonPin, INPUT_PULLDOWN);

    
    scale.set_gain(64);
    scale.begin(SDA,SCL);
    scale.tare();          // Resets the scale to 0.

    delay(1000); //Take some time to open up the Serial Monitor
}

void loop()
{
    long now = millis();

    /*
    // Check if User Reset is triggered by Button
    if (digitalRead(resetButtonPin) == HIGH) {
        delay(50); // Debouncing delay
        if (digitalRead(resetButtonPin) == HIGH) {
            resetButtonPressed = true;
        }
    } else {
        // Reset if the button was pressed
        if (resetButtonPressed) {
            Serial.println("Reset button pressed. Resetting...");
            delay(100); // Delay for stability
            ESP.restart(); // Trigger a software reset
        }
        resetButtonPressed = false;
    }

    */
    
    //Serial.println(counter);
    //Serial.println("active mode");

    

    if (resetDSTimer)
    {
        timerRestart(Timer0_Cfg);
        Serial.println("Reset DST"); // Reset Deep Sleep Timer
        resetDSTimer = false;
    }


    /*
    Get Value from weight Cell here in if loop the code is waiting 20 ms to send a new value
    within this 20 ms a new value shall be fetched from the weight cell
    */

    if (sendData) {
        char sens_str[8];
        /*
        if (!bSensorValue){
          digitalWrite(led1PIN, HIGH);
          float sens = scale.get_value(1)/(calibration_factor);
          dtostrf(sens, 1, 2, sens_str);
          // Store timestamp as binary data (unsigned long)
          digitalWrite(led1PIN, LOW);
          }
        */
        if (now - lastMsg > 160) { // former 16 for ~80 Hz resoulution 
            digitalWrite(led1PIN, HIGH);
            float sens = scale.get_value(1)/(calibration_factor);
            dtostrf(sens, 1, 2, sens_str);
            // Store timestamp as binary data (unsigned long)
            digitalWrite(led1PIN, LOW);
            // Convert the sensor value to a string
            //String dataToSend = String(sensorValue);
            // Code to read sensor data and send it to the connected client
            // Replace this with your sensor data transmission logic
            digitalWrite(ledPIN, HIGH);
            pSensorCharacteristic->setValue(sens_str);
            pSensorCharacteristic->notify();
            digitalWrite(ledPIN, LOW);
            lastMsg = now;
            bSensorValue = false;

            Serial.println(sens); // Reset Deep Sleep Timer
        }
    }
    else if (!sendData)
    {
        // Save Energy
        // Shut down Scale
        // Further Measures to save energy
        Serial.println("Passive Moode - not sending but providing");

    }
    

}


