#include <Arduino.h>
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"


// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Settings
TickType_t cs_wait = 250;  // Time spent in critical section (ms)
TickType_t med_wait = 5000; // Time medium task spends working (ms)

// 2 Define all function
void timer0_callback(void* arg);
void timer1_callback(void* arg);
void initialize_timer();

// Define the timer handle
esp_timer_handle_t timer0_handle;
esp_timer_handle_t timer1_handle;
TimerHandle_t Fire_alarm_TimerHandle = NULL;

// task handler
TaskHandle_t pir_sensor_Handle;
TaskHandle_t led_strigger_Handle;
TaskHandle_t dc_motor_Handle;
TaskHandle_t reset_button_Handle;

//****************************************GPIO PINS*************************************

#define PIR_SENSOR_PIN 4  // infrared sensor pin
#define LED_PIN 2   // led pin
// Define the reset button pin
#define RESET_BUTTON_PIN 14
#define FRQ 1000    // PWM frequency
#define PIN_ANALOG_IN 15  // Potentiometer
#define CHANNEL 0
#define BUTTON_PRESS_DELAY_MS 2000 //Length of button press
#define REVERSE_SWITCH 14
#define DRIVER_PUL 18
#define DRIVER_DIR 19
#define stepsPerRevolution 800
int pd = 500;

//************************************************************* Globals ****************************************
// Defin pins

//******************************************** Help Functions ****************************************


void run_motor(){
  
}

void drive_the_motor_to_open_door(){
  int adcVal = analogRead(PIN_ANALOG_IN); // read adc
  int pwmVal = adcVal;
  Serial.println(pwmVal);
  // run_motor(pwmVal);
}

//************************************* Tasks ************************************************

// Task H (high priority)
void doTaskH(void *parameters) {
  while (1) {
    // Set the spinning direction clockwise:
  digitalWrite(DRIVER_DIR, HIGH);

  // Spin the stepper motor 1 revolution slowly:
  for (int i = 0; i < 50*stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(DRIVER_PUL, HIGH);
    delayMicroseconds(150);
    digitalWrite(DRIVER_PUL, LOW);
    delayMicroseconds(150);    
  }
  delay(1000);

  // digitalWrite(DRIVER_DIR, LOW);

  // // Spin the stepper motor 1 revolution slowly:
  // for (int i = 0; i < 10*stepsPerRevolution; i++) {
  //   // These four lines result in 1 step:
  //   digitalWrite(DRIVER_PUL, HIGH);
  //   delayMicroseconds(pd);
  //   digitalWrite(DRIVER_PUL, LOW);
  //   delayMicroseconds(pd);
  // }
  // delay(1000);



  }
}



//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)
void setup() {
  // Configure Serial
  Serial.begin(921600);
  // GPIO PIN
  // pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  // need to fix this
  // set the maximum speed and initial speed.
  
  // set up stepper motor
  pinMode(DRIVER_DIR, OUTPUT);
  pinMode(DRIVER_PUL, OUTPUT);
      // Create the fire alarm timer   
    
  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Create semaphores and mutexes before starting tasks
  
  // The order of starting the tasks matters to force priority inversion

  

  // Introduce a delay to force priority inversion
  vTaskDelay(1 / portTICK_PERIOD_MS);

  // Start Motor Task (high priority)
  xTaskCreatePinnedToCore(doTaskH,
                          "Task H",
                          1024,
                          NULL,
                          3,
                          &dc_motor_Handle,
                          app_cpu);

  // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop() {
  // Execution should never get here
}

