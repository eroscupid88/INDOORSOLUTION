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

uint32_t Freq = 0;
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
#define WAIT_TIME 200 // 0.2 seconds
#define BUTTON_PIN 14
#define FRQ 1000    // PWM frequency
#define PIN_ANALOG_IN 15  // Potentiometer
#define PIN_R_PWM_EN 32   // R_EN
#define PIN_L_PWM_EN 33   // L_EN
#define CHANNEL 0
#define PIN_LPWM 13
#define PIN_RPWM 12  // drive PWM
#define BUTTON_PRESS_DELAY_MS 2000 //Length of button press

//************************************************************* Globals ****************************************
static SemaphoreHandle_t lock;

int sensorMode = 0;
int doorMode = 0;

//******************************************** Help Functions ****************************************
void run_motor(int number){
  ledcWrite(CHANNEL, number);
}
void enable_motor(){
  digitalWrite(PIN_R_PWM_EN, HIGH);
  digitalWrite(PIN_L_PWM_EN,HIGH);
}
void disable_motor(){
  digitalWrite(PIN_R_PWM_EN, LOW);
  digitalWrite(PIN_L_PWM_EN,LOW);
}
void drive_the_motor_to_open_door(){
  enable_motor();
  int adcVal = analogRead(PIN_ANALOG_IN); // read adc
  float pwmVal = adcVal;
  Serial.println(pwmVal/4096);
  run_motor(pwmVal);
}
void drive_the_motor_to_hold_door(){
  Serial.println("holllllllllllllllllllllllllllllllllllllllllllldddddddddddddddddddddddddddd");
  enable_motor();
  run_motor(700);
}

void initialize_timer(){
  //     TIMER 0
  esp_timer_create_args_t timer0_args = {.callback = timer0_callback,.name = "timer0"};
  esp_timer_create(&timer0_args, &timer0_handle);
  
  //     TIMER 1 
  esp_timer_create_args_t timer1_args = {.callback = timer1_callback,.name = "timer1"};
  esp_timer_create(&timer1_args, &timer1_handle);
}

/************************************TIMERS************************************************************/
// Callback function for TIMER0 when Door is in wide open 
void timer0_callback(void* arg) {
    Serial.println("                                        TIMER0 triggered  8s  ");
    esp_timer_start_once(timer1_handle, 1000000);  // 4 seconds
    doorMode = 2;
}
// Callback function for TIMER1 when Door is release (auto closer work now)
void timer1_callback(void* arg) {
    Serial.println("                                        TIMER1 triggered  4s  ");
    doorMode = 3;
}

//************************************* Tasks ************************************************


//********************************RESET BUTTON************************************************
TaskHandle_t reset_trigger_Handle;
TimerHandle_t reset_timer_Handle;
bool reset_mode = true;
void resetTimerCallback(TimerHandle_t xTimer)
{
  // Stop the timer and notify the long press task
  xTimerStop(reset_timer_Handle, 0);
  xTaskNotifyGive(reset_trigger_Handle);
}

void reset_trigger_task(void *pvParameters)
{
  while(1)
  {
    uint32_t ulNotificationValue;
    Freq = getCpuFrequencyMhz();
    // Wait for a notification from the timer
    ulNotificationValue = ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(100));

    if (ulNotificationValue != 0) {
    // A notification was received, process it
        
        reset_mode = !reset_mode;
        if(reset_mode){
          disable_motor();
          vTaskSuspend(dc_motor_Handle); // Suspend the task
          // vTaskSuspend(pir_sensor_Handle);
          doorMode = 3;
          esp_timer_stop(timer0_handle);
          esp_timer_stop(timer1_handle);
          
        }else{
          
          enable_motor();
          xTaskResumeFromISR(dc_motor_Handle);   // Resume the task from ISR
          // xTaskResumeFromISR(pir_sensor_Handle); // Resume the task from ISR
          doorMode = 0;
        }

    } else {
        // A timeout occurred
        if (reset_mode){
            Serial.println("DISABLE");
            // Serial.println(Freq);
        } else {
            // Serial.println(Freq);
            // Serial.println("                                    ENABLE");
        }
                 
    }
    
  }
}

void buttonInterrupt()
{
  if (digitalRead(BUTTON_PIN) == LOW)
  {
    // Start the timer
    xTimerStart(reset_timer_Handle, 0);

  }
  else
  {
    // Button released before 2 seconds
    xTimerStop(reset_timer_Handle, 0);
  }
}

//********************************************************************************************




// void resetButtonTask(void *pvParameters){
//   pinMode(RESET_BUTTON_PIN, INPUT_PULLUP); // Set the reset button pin as input with pull-up resistor
  
//   bool isTaskSuspended = true; // Initialize a flag to keep track of the task's state

//   while (1)
//   {
//     if (digitalRead(RESET_BUTTON_PIN) == LOW) // If the reset button is pressed
//     {
//       if (isTaskSuspended) // If the task is currently suspended, resume it
//       {
//         xTaskResumeFromISR(dc_motor_Handle); // Resume the task from ISR
//         xTaskResumeFromISR(pir_sensor_Handle); // Resume the task from ISR
//         doorMode = 0;
//         isTaskSuspended = false; // Update the task state flag
//         Serial.println("                              Resume the MOTOR TASK");
//       }
//       else // If the task is currently running, suspend it
//       {
//         vTaskSuspend(dc_motor_Handle); // Suspend the task
//         vTaskSuspend(pir_sensor_Handle);
//         doorMode = 3;
//         esp_timer_stop(timer0_handle);
//         esp_timer_stop(timer1_handle);
//         isTaskSuspended = true; // Update the task state flag
//         Serial.println("                              Suspense the MOTOR TASK");
//       }
//       delay(100); // Wait for 1 s to debounce the button
//     }
//     else
//     {
//       delay(50); // Wait for 0.5s before checking the button state again
//     }
//   }
// }



// Task L (low priority)
void doTaskL(void *parameters) {

  TickType_t timestamp;

  // Do forever
  while (1) {
    xSemaphoreTake(lock, portMAX_DELAY);
    // Say how long we spend waiting for a lock
    Serial.print("Task Sensor got lock. Spent ");
    // Hog the processor for a while doing nothing
    timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    // while ( (xTaskGetTickCount() * portTICK_PERIOD_MS) - timestamp < cs_wait);

    // do something over here
    if (digitalRead(PIR_SENSOR_PIN) == HIGH && doorMode == 0){
      sensorMode = 1;
      doorMode = 1;
    }
    else if (digitalRead(PIR_SENSOR_PIN) == HIGH && doorMode == 5 ){
      // reset timer 1 if sensor is activate when door is on hold
      sensorMode = 1;
      Serial.print("                                          RESET timer 1");
      esp_timer_stop(timer1_handle);
      esp_timer_start_once(timer1_handle, 1000000); // 1/32 seconds
    }
    else {
      sensorMode = 0;
    }
    xSemaphoreGive(lock);

    // Go to sleep
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

// Task M (medium priority)
void doTaskM(void *parameters) {
  while (1) {
    
    if(sensorMode == 1){
      Serial.print("Task LED detect a sensor >>>>>>> turn the LED on");
      digitalWrite(LED_PIN,HIGH);
    }
    else if(sensorMode ==0){
      digitalWrite(LED_PIN,LOW);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Task H (high priority)
void doTaskH(void *parameters) {
  while (1) {
    Serial.println("Motor Task DOING SOME WORK...");
    xSemaphoreTake(lock, portMAX_DELAY);
    // MOTOR DOING SOME WORK HERE
    if (doorMode == 0){
      disable_motor();
    }
    else if (doorMode == 1){
      // drive_the_motor_to_hold_door();
      drive_the_motor_to_open_door();
      esp_timer_start_once(timer0_handle, 1000000);  // 1 seconds
    }
    else if (doorMode == 2){
      // drive_the_motor_to_hold_door();
      drive_the_motor_to_open_door();
    }
    else{
      Serial.println("disable#####");
      disable_motor();
      vTaskDelay(5000 / portTICK_PERIOD_MS); // 5s
      doorMode = 0;
    }
    Serial.print("                            Sensor Mode is : ");
    Serial.println(sensorMode);
    Serial.print("                            Door Mode is : ");
    Serial.println(doorMode);
    // Release lock
    xSemaphoreGive(lock);
    
    // Go to sleep
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {
  // Configure Serial
  Serial.begin(921600);

  initialize_timer();
  // GPIO PIN
  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_R_PWM_EN, OUTPUT);
  pinMode(PIN_L_PWM_EN, OUTPUT);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  disable_motor();
  ledcSetup(CHANNEL, 2000, 12);
  ledcAttachPin(PIN_RPWM, CHANNEL);
      // Create the fire alarm timer 

  pinMode(BUTTON_PIN, INPUT_PULLUP); 
    
  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Create semaphores and mutexes before starting tasks
  lock = xSemaphoreCreateMutex();
  
  // The order of starting the tasks matters to force priority inversion

  
  // Start Button Task (low priority)
  xTaskCreatePinnedToCore(reset_trigger_task, "reset_trigger_Task", 2048, NULL, configMAX_PRIORITIES - 1, &reset_trigger_Handle,app_cpu);
  
  // Create the timer
  reset_timer_Handle = xTimerCreate("resetTimer", WAIT_TIME / portTICK_PERIOD_MS, pdFALSE, 0, resetTimerCallback);
  
  // Attach the button interrupt
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInterrupt, CHANGE);
  // xTaskCreatePinnedToCore(resetButtonTask, "resetButtonTask", 10000, NULL, configMAX_PRIORITIES - 1, NULL, app_cpu); // Create the reset button task with the highest priority
  // Start Sensor Task (low priority)
  xTaskCreatePinnedToCore(doTaskL,
                          "Task L",
                          1024,
                          NULL,
                          4,
                          &pir_sensor_Handle,
                          app_cpu);

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

  // Start LED Task (medium priority)
  xTaskCreatePinnedToCore(doTaskM,
                          "Task M",
                          1024,
                          NULL,
                          2,
                          &led_strigger_Handle,
                          app_cpu);


  // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop() {
  // Execution should never get here
}

