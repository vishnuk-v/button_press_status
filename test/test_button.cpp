
#include "main.h"

void test_pwr_button_count(){
pwr_button_count();
}

void test_ble_on_off()
{
    ble_button_on_off();
}

void test_tmr_counter(){
    timer_control();
}
void setup()
{
    Serial.begin(115200);
    UNITY_BEGIN();
    RUN_TEST(test_pwr_button_count);
    RUN_TEST(test_ble_on_off);
    RUN_TEST(test_tmr_counter);
    UNITY_END();
}

void loop()
{
    //
}