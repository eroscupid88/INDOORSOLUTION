#include <unity.h>
#include "main.h"

// Replace with the pins used for testing
#define PIR_SENSOR_PIN_TEST 23
#define LED_PIN_TEST 22
#define RESET_BUTTON_PIN_TEST 21
#define PIN_ANALOG_IN_TEST 33
#define PIN_R_PWM_EN_TEST 26
#define PIN_L_PWM_EN_TEST 27
#define CHANNEL_TEST 0
#define PIN_LPWM_TEST 13
#define PIN_RPWM_TEST 12
#define BUTTON_PRESS_DELAY_MS_TEST 2000

// Declare variables that are used in the main code
extern SemaphoreHandle_t lock;
extern int sensorMode;
extern int doorMode;

void test_timer_callbacks(void) {
    // Test timer0_callback
    doorMode = 0;
    timer0_callback(NULL);
    TEST_ASSERT_EQUAL_INT(2, doorMode);
    
    // Test timer1_callback
    doorMode = 0;
    timer1_callback(NULL);
    TEST_ASSERT_EQUAL_INT(3, doorMode);
}

void test_reset_button(void) {
    // Initialize the reset button
    pinMode(RESET_BUTTON_PIN_TEST, INPUT_PULLUP);
    attachInterrupt(RESET_BUTTON_PIN_TEST, reset_button_isr, FALLING);
    reset_timer_Handle = xTimerCreate("resetTimer", pdMS_TO_TICKS(BUTTON_PRESS_DELAY_MS_TEST), pdFALSE, 0, resetTimerCallback);

    // Test pressing the reset button
    reset_mode = false;
    reset_button_isr();
    TEST_ASSERT_EQUAL_INT(true, reset_mode);

    // Test releasing the reset button
    reset_mode = true;
    reset_button_isr();
    TEST_ASSERT_EQUAL_INT(false, reset_mode);
}

void test_dc_motor(void) {
    // Test run_motor function
    run_motor(500);
    TEST_ASSERT_EQUAL_INT(500, ledcRead(CHANNEL_TEST));
    
    // Test enable_motor function
    enable_motor();
    TEST_ASSERT_EQUAL_INT(HIGH, digitalRead(PIN_R_PWM_EN_TEST));
    TEST_ASSERT_EQUAL_INT(HIGH, digitalRead(PIN_L_PWM_EN_TEST));
    
    // Test disable_motor function
    disable_motor();
    TEST_ASSERT_EQUAL_INT(LOW, digitalRead(PIN_R_PWM_EN_TEST));
    TEST_ASSERT_EQUAL_INT(LOW, digitalRead(PIN_L_PWM_EN_TEST));
    
    // Test drive_the_motor_to_open_door function
    analogWrite(PIN_ANALOG_IN_TEST, 2048);
    drive_the_motor_to_open_door();
    TEST_ASSERT_EQUAL_INT(2048/4096*1023, ledcRead(CHANNEL_TEST));
    
    // Test drive_the_motor_to_hold_door function
    drive_the_motor_to_hold_door();
    TEST_ASSERT_EQUAL_INT(700, ledcRead(CHANNEL_TEST));
}

void setup() {
    UNITY_BEGIN();
    
    // Run tests
    RUN_TEST(test_timer_callbacks);
    RUN_TEST(test_reset_button);
    RUN_TEST(test_dc_motor);
    
    UNITY_END();
}

void loop() {

}
