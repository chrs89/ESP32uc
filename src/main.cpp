#include <Arduino.h>
#define seconds_to_trigger_DeepSleep 3600 // Timer in seconds until sleep mode is triggered 

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
// Bluetooth Serial object
BluetoothSerial SerialBT;

#define timer_prescale 64000
#define timer1_prescale 80

#define ledPIN 12

hw_timer_t *Timer1_Cfg = NULL;


hw_timer_t *Timer0_Cfg = NULL;
int32_t counter = 0;
RTC_DATA_ATTR int bootCount = 0;
 
void IRAM_ATTR Timer0_ISR()
{
    counter++;
    esp_deep_sleep_start();
    
}

void IRAM_ATTR Timer1_ISR()
{
  counter++;
  digitalWrite(ledPIN,!digitalRead(ledPIN));
    
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
    
    delay(1000); //Take some time to open up the Serial Monitor
    
    // Initialize Timer Interrupt
    Serial.println("Interrupt Initialized");
    Timer0_Cfg = timerBegin(0,timer_prescale, true); // Prescaler set to 80*1e6 which means timer ticks every second
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 1250*seconds_to_trigger_DeepSleep, true);
    timerAlarmEnable(Timer0_Cfg);

    Timer1_Cfg = timerBegin(1,timer1_prescale, true); // Prescaler set to 80*1e6 which means timer ticks every second
    timerAttachInterrupt(Timer1_Cfg, &Timer1_ISR, true);
    timerAlarmWrite(Timer1_Cfg, 1000, true);
    timerAlarmEnable(Timer1_Cfg);

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
    SerialBT.begin("ESP32_one");       //Bluetooth device name 
    Serial.println("The device started, now you can pair it with bluetooth!");

    pinMode(ledPIN, OUTPUT);
}

void loop()
{
    delay(1000);
    Serial.println(counter);
    Serial.println("active mode");

    if (bootCount == 10)
    {
        timerRestart(Timer0_Cfg);
    }
}