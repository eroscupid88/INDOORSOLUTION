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
#include <ESP32Servo.h>

#define app_cpu 0
// define task
static TaskHandle_t task1 = NULL;
static TaskHandle_t task2 = NULL;

#define PIN_LED 2
#define PIN_BUTTON 13

const char msg[] = "GE495 capstone project running demo";
void startTask1 (void *parameter){
  while (1)
  {
    Serial.println("Task1");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
void startTask2 (void *parameter){

    while (1){
      Serial.println("Task2");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


void setup() {
  Serial.begin(921600);
  Serial.println("FREE RTOS Start");

  xTaskCreatePinnedToCore(startTask1,
                          "task 1",
                          1024,
                          NULL,
                          2,
                          &task1,
                          app_cpu );

  xTaskCreatePinnedToCore(startTask2,
                          "task 2",
                          1024,
                          NULL,
                          3,
                          &task2,
                          app_cpu );
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);
}

void loop() {
}







