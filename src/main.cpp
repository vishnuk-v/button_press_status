#include <Arduino.h>
#define POWER_BTN                48                // to turn on/off the device and change the RPM 
#define BLE_BTN                  46                // to turn on/off the BLE 
#define TIMER_CONTROL_BTN        45                // to set the timer values 5/10/15/20/25
int pwr_btn_counter =            0;                // to count the number of button pushes 
int pwr_btn_state =              0;                // to determine the current button state 
int pwr_btn_last_state =         0;                // to determine the previous button state
int pwr_btn_cutoff =             4;                // to turn off the motor when button is pushed 4 times 
int ble_btn_state =              0;                // current ble button state
int ble_btn_last_state =         0;                // previous button state
int tmr_btn_counter =            0;
int tmr_btn_state =              0;
int tmr_btn_last_state =         0;


// to count the power button presses
void pwr_button_count()
{
  pwr_btn_state = digitalRead(POWER_BTN);
  if (pwr_btn_state != pwr_btn_last_state)
  {
    if(pwr_btn_state == HIGH)
    {
      digitalWrite(TIMER_CONTROL_BTN,HIGH);     // to turn on the 5 minutes timer and start rotating
      // write LED logic here to turn on GREEN LED, power on the main module  
      pwr_btn_counter++;                          // to increase the speed and stop the motor 
        
      if(pwr_btn_counter == pwr_btn_cutoff )
      {
        
        digitalWrite(TIMER_CONTROL_BTN,LOW);  // motor stop digitalWrite(MOTORpin,LOW);
        pwr_btn_counter = 0;

      }
    }
    else{
      //  digitalWrite(MOTORPIN, LOW);      
    }
  }
  pwr_btn_last_state = pwr_btn_state;                 // last button state is updated to the current button state
}

// to turn on and off the ble to connect to a client device, press and hold to turn on the device and vice versa 

void ble_button_on_off(){
  ble_btn_state = digitalRead(BLE_BTN);
  if(ble_btn_state != ble_btn_last_state)
  {
    if(ble_btn_state == HIGH)
    {
      // BLE on and start advertising 
    }
  }
  ble_btn_last_state = ble_btn_state;

}

// timer control each press will increase the timer value by 5 minutes 

void timer_control()
{
  tmr_btn_state = digitalRead(TIMER_CONTROL_BTN);
  if(tmr_btn_state != tmr_btn_last_state)
  {
    if(ble_btn_state == HIGH)
    {
      tmr_btn_counter++;
      // timer function and turn off the motor when the timer is zero
    }
  }
  tmr_btn_last_state = tmr_btn_state;
}

void setup()
{
  //enable serial communication
  Serial.begin(115200);

  // set the power, ble and timer pins
  pinMode(POWER_BTN, INPUT_PULLUP);
  pinMode(BLE_BTN,INPUT_PULLUP);
  pinMode(TIMER_CONTROL_BTN,INPUT_PULLUP);

}

void loop()
{
  // int pwr_btn, ble_btn, tim_btn;
  // // pwr_btn 
  pwr_button_count();
  ble_button_on_off();
  timer_control();




  // Serial.println(pwr_btn);
  // if(POWER_BTN == HIGH)
  // {
    
  // }
  // ble_btn=digitalRead(BLE_BTN);
  // Serial.println(ble_btn);
  // tim_btn= digitalRead(TIMER_CONTROL_BTN);
  // Serial.println(tim_btn);

 
}