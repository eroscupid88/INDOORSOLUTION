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
#include <Stepper.h>


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

// Motor Connections (unipolar motor driver)
const int In1 = 5;
const int In2 = 18;
const int In3 = 19;
const int In4 = 21;

// Number of steps per internal motor revolution 
const float STEPS_PER_REV = 32; 
 
//  Amount of Gear Reduction
const float GEAR_RED = 64;
 
// Number of steps per geared output rotation
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;
 
// Define Variables
 
// Number of Steps Required
int StepsRequired;
Stepper steppermotor(STEPS_PER_REV, In1, In3, In2, In4);

//************************************************************* Globals ****************************************
static SemaphoreHandle_t lock;
// AccelStepper myStepper(AccelStepper::FULL4WIRE, In1, In3, In2, In4); 

int sensorMode = 0;
int doorMode = 0;

//******************************************** Help Functions ****************************************

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
    esp_timer_start_once(timer1_handle, 4000000);  // 4 seconds
    doorMode = 2;
}
// Callback function for TIMER1 when Door is release (auto closer work now)
void timer1_callback(void* arg) {
    Serial.println("                                        TIMER1 triggered  4s  ");
    doorMode = 3;
}

//************************************* Tasks ************************************************

void resetButtonTask(void *pvParameters)
{
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP); // Set the reset button pin as input with pull-up resistor
  
  bool isTaskSuspended = true; // Initialize a flag to keep track of the task's state

  while (1)
  {
    if (digitalRead(RESET_BUTTON_PIN) == LOW) // If the reset button is pressed
    {
      if (isTaskSuspended) // If the task is currently suspended, resume it
      {
        xTaskResumeFromISR(dc_motor_Handle); // Resume the task from ISR
        xTaskResumeFromISR(pir_sensor_Handle); // Resume the task from ISR
        doorMode = 0;
        isTaskSuspended = false; // Update the task state flag
        Serial.println("                              Resume the MOTOR TASK");
      }
      else // If the task is currently running, suspend it
      {
        vTaskSuspend(dc_motor_Handle); // Suspend the task
        vTaskSuspend(pir_sensor_Handle);
        doorMode = 3;
        esp_timer_stop(timer0_handle);
        esp_timer_stop(timer1_handle);
        isTaskSuspended = true; // Update the task state flag
        Serial.println("                              Suspense the MOTOR TASK");
      }
      delay(100); // Wait for 1 s to debounce the button
    }
    else
    {
      delay(50); // Wait for 0.5s before checking the button state again
    }
  }
}



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
    else if (digitalRead(PIR_SENSOR_PIN) == HIGH && doorMode == 2){
      // reset timer 1 if sensor is activate when door is on hold
      sensorMode = 1;
      Serial.print("                                          RESET timer 1");
      esp_timer_stop(timer1_handle);
      esp_timer_start_once(timer1_handle, 4000000); // 4 seconds
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
    // xSemaphoreTake(lock, portMAX_DELAY);
    // MOTOR DOING SOME WORK HERE
    steppermotor.setSpeed(1);    
    StepsRequired  =  4;
    steppermotor.step(StepsRequired);
    delay(2000);
  
    // Rotate CW 1/2 turn slowly
    StepsRequired  =  STEPS_PER_OUT_REV / 2; 
    steppermotor.setSpeed(300);   
    steppermotor.step(StepsRequired);
    delay(1000);
    
    // Rotate CCW 1/2 turn quickly
    StepsRequired  =  - STEPS_PER_OUT_REV / 2;   
    steppermotor.setSpeed(1000);  
    steppermotor.step(StepsRequired);
    delay(2000);
    // Release lockZ
    // xSemaphoreGive(lock);
    
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
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  ledcSetup(CHANNEL, 1000, 12);

  // need to fix this
  // ledcAttachPin(PIN_RPWM, CHANNEL);
      // Create the fire alarm timer 

  // stepper motor
  // myStepper.setMaxSpeed(100.0);
  // myStepper.setSpeed(50);
  // myStepper.moveTo(2000);
  
    
  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Create semaphores and mutexes before starting tasks
  lock = xSemaphoreCreateMutex();
  
  // The order of starting the tasks matters to force priority inversion

  
  // Start Button Task (low priority)
  xTaskCreatePinnedToCore(resetButtonTask, "resetButtonTask", 10000, NULL, configMAX_PRIORITIES - 1, NULL, app_cpu); // Create the reset button task with the highest priority
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

