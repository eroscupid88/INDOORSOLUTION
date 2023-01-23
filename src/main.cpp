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
#define app_cpu 0


// define task
static TaskHandle_t task1 = NULL;
static TaskHandle_t task2 = NULL;

const char msg[] = "GE495 capstone project running demo";
void startTask1 (void *parameter){
    int msg_len = strlen(msg);
    while (1){
      Serial.println();
      for (int i = 0; i < msg_len;i++){
        Serial.print(msg[i]);

      }
      Serial.println();
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void startTask2 (void *parameter){

    while (1){
      Serial.print("*");
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup() {
  Serial.begin(921600);
  Serial.println("FREE RTOS Start");
  Serial.print(xPortGetCoreID());
  Serial.print("with Priority : ");
  Serial.println(uxTaskPriorityGet(NULL));
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
}

void loop() {
  Serial.println("Start");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  for (int i = 0; i < 3; i++)
  {
      vTaskSuspend(task2);
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      vTaskResume(task2);
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      }

      if (task1 !=NULL){
      vTaskDelete(task1);
      task1 = NULL;
      }
      // test
}