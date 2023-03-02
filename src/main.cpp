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
#include <ESP32Servo.h>


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

// Define the timer handle
esp_timer_handle_t timer0_handle;
esp_timer_handle_t timer1_handle;

//****************************************GPIO PINS*************************************

#define PIR_SENSOR_PIN 4  // infrared sensor pin
#define LED_PIN 2   // led pin
// Define the reset button pin
#define RESET_BUTTON_PIN 14
#define FRQ 1000    // PWM frequency
#define PIN_ANALOG_IN 15  // Potentiometer
#define PIN_R_PWM_EN 32   // R_EN
#define PIN_L_PWM_EN 33   // L_EN
#define CHANNEL 0
#define PIN_LPWM 19
#define PIN_RPWM 21

//************************************************************* Globals ****************************************
static SemaphoreHandle_t lock;

//******************************************** Help Functions ****************************************
void run_motor(int number){
    ledcWrite(CHANNEL, number);
}

/************************************TIMERS************************************************************/
// Callback function for TIMER0 when Door is in wide open 
void timer0_callback(void* arg) {
    Serial.println("                                        TIMER0 triggered");
    esp_timer_start_periodic(timer1_handle, 1000000);  // 6 seconds
}
// Callback function for TIMER1 when Door is release (auto closer work now)
void timer1_callback(void* arg) {
    Serial.println("                                        TIMER1 triggered");
}

//************************************* Tasks ************************************************
// Task L (low priority)
void doTaskL(void *parameters) {

  TickType_t timestamp;

  // Do forever
  while (1) {

    // Take lock
    Serial.println("Task L trying to take lock...");
    timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xSemaphoreTake(lock, portMAX_DELAY);
    esp_timer_start_periodic(timer0_handle, 5000000);  // 6 seconds
    // Say how long we spend waiting for a lock
    Serial.print("Task L got lock. Spent ");
    Serial.print((xTaskGetTickCount() * portTICK_PERIOD_MS) - timestamp);
    Serial.println(" ms waiting for lock. Doing some work...");

    // Hog the processor for a while doing nothing
    timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    // while ( (xTaskGetTickCount() * portTICK_PERIOD_MS) - timestamp < cs_wait);

    // Release lock
    Serial.println("Task L releasing lock.");
    xSemaphoreGive(lock);

    // Go to sleep
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

// Task M (medium priority)
void doTaskM(void *parameters) {

  TickType_t timestamp;

  // Do forever
  while (1) {

    // Hog the processor for a while doing nothing
    Serial.println("Task M doing some work...");
    timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    while ( (xTaskGetTickCount() * portTICK_PERIOD_MS) - timestamp < med_wait);

    // Go to sleep
    Serial.println("Task M done!");
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Task H (high priority)
void doTaskH(void *parameters) {

  TickType_t timestamp;

  // Do forever
  while (1) {

    // Take lock
    Serial.println("Task H trying to take lock...");
    timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xSemaphoreTake(lock, portMAX_DELAY);

    // Say how long we spend waiting for a lock
    Serial.print("Task H got lock. Spent ");
    Serial.print((xTaskGetTickCount() * portTICK_PERIOD_MS) - timestamp);
    Serial.println(" ms waiting for lock. Doing some work...");

    // Hog the processor for a while doing nothing
    timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    // while ( (xTaskGetTickCount() * portTICK_PERIOD_MS) - timestamp < cs_wait){
    //   Serial.println("***************************************************");
    // };

    // Release lock
    Serial.println("Task H releasing lock.");
    xSemaphoreGive(lock);
    
    // Go to sleep
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {
  // Configure Serial
  Serial.begin(921600);
  esp_timer_create_args_t timer0_args = {.callback = timer0_callback,.name = "timer0"};
  esp_timer_create(&timer0_args, &timer0_handle);
  
  // activate timer 1 to start hold position(force is equal to auto closer)
  esp_timer_create_args_t timer1_args = {.callback = timer1_callback,.name = "timer1"};
  esp_timer_create(&timer1_args, &timer1_handle);

  // GPIO PIN
  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_RPWM, OUTPUT);
  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Create semaphores and mutexes before starting tasks
  lock = xSemaphoreCreateMutex();
  
  // The order of starting the tasks matters to force priority inversion

  // Start Sensor Task (low priority)
  xTaskCreatePinnedToCore(doTaskL,
                          "Task L",
                          1024,
                          NULL,
                          1,
                          NULL,
                          app_cpu);

  // Introduce a delay to force priority inversion
  vTaskDelay(1 / portTICK_PERIOD_MS);

  // Start Motor Task (high priority)
  xTaskCreatePinnedToCore(doTaskH,
                          "Task H",
                          1024,
                          NULL,
                          3,
                          NULL,
                          app_cpu);

  // Start LED Task (medium priority)
  xTaskCreatePinnedToCore(doTaskM,
                          "Task M",
                          1024,
                          NULL,
                          2,
                          NULL,
                          app_cpu);


  // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop() {
  // Execution should never get here
}

