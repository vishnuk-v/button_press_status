#include <Arduino.h>
#include <unity.h>

#define POWER_BTN                48  // Pin for the power button
#define BLE_BTN                  46  // Pin for the BLE button
#define TIMER_CONTROL_BTN        45  // Pin for the timer control button

int pwr_btn_counter = 0;             // Counter for power button presses
volatile int pwr_btn_state = 0;      // Current power button state
volatile int pwr_btn_last_state = 0; // Previous power button state
int pwr_btn_cutoff = 4;              // Number of presses to turn off the motor

volatile int ble_btn_state = 0;      // Current BLE button state
volatile int ble_btn_last_state = 0; // Previous BLE button state

int tmr_btn_counter = 0;             // Counter for timer button presses
volatile int tmr_btn_state = 0;      // Current timer button state
volatile int tmr_btn_last_state = 0; // Previous timer button state

unsigned long lastDebounceTime = 0;  // Last time a button state changed
unsigned long debounceDelay = 50;    // Debounce delay time in milliseconds

unsigned long pwr_btn_press_time = 0;// Time when the power button was pressed
unsigned long ble_btn_press_time = 0;// Time when the BLE button was pressed

bool device_on = false;              // Device on/off state
bool ble_on = false;                 // BLE on/off state

volatile bool pwr_btn_flag = false;  // Flag for power button interrupt
volatile bool ble_btn_flag = false;  // Flag for BLE button interrupt
volatile bool tmr_btn_flag = false;  // Flag for timer button interrupt

// Interrupt Service Routines (ISR)
void IRAM_ATTR powerButtonISR() {
  pwr_btn_flag = true;
}

void IRAM_ATTR bleButtonISR() {
  ble_btn_flag = true;
}

void IRAM_ATTR timerButtonISR() {
  tmr_btn_flag = true;
}

// void setup() {
//   Serial.begin(115200);

//   pinMode(POWER_BTN, INPUT_PULLUP);
//   pinMode(BLE_BTN, INPUT_PULLUP);
//   pinMode(TIMER_CONTROL_BTN, INPUT_PULLUP);

//   // Initialize states
//   pwr_btn_last_state = digitalRead(POWER_BTN);
//   ble_btn_last_state = digitalRead(BLE_BTN);
//   tmr_btn_last_state = digitalRead(TIMER_CONTROL_BTN);

//   // Attach interrupts
//   attachInterrupt(digitalPinToInterrupt(POWER_BTN), powerButtonISR, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(BLE_BTN), bleButtonISR, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(TIMER_CONTROL_BTN), timerButtonISR, CHANGE);
// }

void pwr_button_count() {
  if (pwr_btn_flag) {
    pwr_btn_flag = false;

    pwr_btn_state = digitalRead(POWER_BTN);
    if (pwr_btn_state != pwr_btn_last_state) {      // to check for press or noise 
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (pwr_btn_state != pwr_btn_last_state) {
        if (pwr_btn_state == HIGH) {
            TEST_ASSERT_BIT_HIGH(pwr_btn_state,HIGH);
          pwr_btn_press_time = millis();  // Record the time the button was pressed
        } else {
          if ((millis() - pwr_btn_press_time) >= 2000) {  // If held for 2 seconds or more
            // Turn off the device
            device_on = false;
            digitalWrite(TIMER_CONTROL_BTN, LOW);
            TEST_ASSERT_BIT_LOW(device_on,false);
            pwr_btn_counter = 0;
            Serial.println("Device turned off");
          } else {
            if (!device_on) {
              // Turn on the device
              device_on = true;
              digitalWrite(TIMER_CONTROL_BTN, HIGH);
              TEST_ASSERT_BIT_HIGH(device_on, true);
              Serial.println("Device turned on");
            } else {
              pwr_btn_counter++;
              if (pwr_btn_counter == pwr_btn_cutoff) {
                TEST_ASSERT_EQUAL(pwr_btn_counter, 4);
                digitalWrite(TIMER_CONTROL_BTN, LOW);  // Motor stop
                pwr_btn_counter = 0;
              }
            }
          }
        }
        pwr_btn_last_state = pwr_btn_state;
      }
    }
  }
}

void ble_button_on_off() {
  if (ble_btn_flag) {
    ble_btn_flag = false;

    ble_btn_state = digitalRead(BLE_BTN);
    if (ble_btn_state != ble_btn_last_state) {
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (ble_btn_state != ble_btn_last_state) {
        if (ble_btn_state == HIGH) {
          ble_btn_press_time = millis();  // Record the time the button was pressed
        } else {
          if ((millis() - ble_btn_press_time) >= 2000) {  // If held for 2 seconds or more
            // Toggle BLE state
            ble_on = !ble_on;
            if (ble_on) {
              // Turn on BLE and start advertising
              TEST_ASSERT_BIT_HIGH(ble_on,true);
              Serial.println("BLE turned on");
            } else {
              // Turn off BLE
              TEST_ASSERT_BIT_LOW(ble_on, false);
              Serial.println("BLE turned off");

            }
          }
        }
        ble_btn_last_state = ble_btn_state;
      }
    }
  }
}

void timer_control() {
  if (tmr_btn_flag) {
    tmr_btn_flag = false;

    tmr_btn_state = digitalRead(TIMER_CONTROL_BTN);
    if (tmr_btn_state != tmr_btn_last_state) {
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (tmr_btn_state != tmr_btn_last_state) {
        if (tmr_btn_state == HIGH) {
            TEST_ASSERT_BIT_HIGH(tmr_btn_state,HIGH);
          tmr_btn_counter++;
          // Timer function and turn off the motor when the timer is zero
          Serial.print("Timer set to ");
          Serial.print(tmr_btn_counter * 5);
          TEST_ASSERT_EQUAL(tmr_btn_counter,1);
          Serial.println(" minutes");
        }
        tmr_btn_last_state = tmr_btn_state;
      }
    }
  }
}

// void loop() {
//   pwr_button_count();
//   ble_button_on_off();
//   timer_control();
// }
