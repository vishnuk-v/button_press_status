#include <Arduino.h>
#define POWER_BTN           48              // to turn on/off the device and change the RPM 
#define BLE_BTN             46              // to turn on/off the BLE 
#define TIMER_CONTROL_BTN   45              // to set the timer values 5/10/15/20/25
void setup()
{
  Serial.begin(115200);
  pinMode(POWER_BTN, INPUT_PULLUP);
  pinMode(BLE_BTN,INPUT_PULLUP);
  pinMode(TIMER_CONTROL_BTN,INPUT_PULLUP);

}

void loop()
{
  int p_btn, ble_btn, tim_btn;
  p_btn = digitalRead(POWER_BTN);
  Serial.println(p_btn);
  ble_btn=digitalRead(BLE_BTN);
  Serial.println(ble_btn);
  tim_btn= digitalRead(TIMER_CONTROL_BTN);
  Serial.println(tim_btn);

 
}