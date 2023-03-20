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
TickType_t cs_wait = 250;   // Time spent in critical section (ms)
TickType_t med_wait = 5000; // Time medium task spends working (ms)

// 2 Define all function
void timer0_callback(void *arg);
void timer1_callback(void *arg);
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

#define PIR_SENSOR_PIN 4 // infrared sensor pin
// Define the reset button pin
#define RESET_BUTTON_PIN 14
#define WAIT_TIME 200 // 0.2 seconds
#define BUTTON_PIN 14
#define FRQ 1000 // PWM frequency
// #define PIN_ANALOG_IN 15  // Potentiometer
#define CHANNEL 0
#define OPEN_DOOR_BUTTON_PIN 22
#define BUTTON_PRESS_DELAY_MS 2000 // Length of button press
#define DRIVER_PUL 18
#define DRIVER_DIR 19
#define RELAY_INPUT 23
#define stepsPerRevolution 3300
#define SOUND_SPEED 0.034 // cm/uS
#define trigPin 2
#define echoPin 15
//************************************************************* Globals ****************************************
static SemaphoreHandle_t lock;

int stepAlreadyOpen = 0;
int sensorMode = 0;
int doorMode = 0;

int min_distance = 15; // Closing distance in cm
int duration;

//******************************************** Help Functions ****************************************

void read_analog()
{
  int adcVal = analogRead(PIN_ANALOG_IN); // read adc
  int pwmVal = adcVal;
  Serial.println(pwmVal);
  // run_motor(pwmVal);
}

void openning_door_signal()
{
  Serial.print("OPEN DOOR PUSHED");
  if (doorMode == 0)
  {
    doorMode = 1;
  }
}

void open_door()
{
  if (doorMode == 2)
  {
    digitalWrite(DRIVER_DIR, LOW);
    // Spin the stepper motor 1 revolution slowly:
    for (int i = 0; i < stepsPerRevolution; i++)
    {
      // These four lines result in 1 step:
      digitalWrite(RELAY_INPUT, HIGH);
      digitalWrite(DRIVER_PUL, HIGH);
      delayMicroseconds(3000);
      digitalWrite(DRIVER_PUL, LOW);
      delayMicroseconds(2400);
      if (digitalRead(PIR_SENSOR_PIN) == HIGH)
      {
        break;
      }
    }
    if (digitalRead(PIR_SENSOR_PIN) == HIGH)
    {
      doorMode = 4;
    }
    else
    {
      doorMode = 3;
    }
    esp_timer_start_once(timer0_handle, 4000000); // 4 seconds
  }

  delay(1000);
}

void initialize_timer()
{
  //     TIMER 0
  esp_timer_create_args_t timer0_args = {.callback = timer0_callback, .name = "timer0"};
  esp_timer_create(&timer0_args, &timer0_handle);

  //     TIMER 1
  esp_timer_create_args_t timer1_args = {.callback = timer1_callback, .name = "timer1"};
  esp_timer_create(&timer1_args, &timer1_handle);
}

/************************************TIMERS************************************************************/
// Callback function for TIMER0 when Door is in wide open
void timer0_callback(void *arg)
{
  Serial.println("                                        TIMER0 triggered  4s  ");
  xSemaphoreTake(lock, portMAX_DELAY);
  doorMode = 4;
  xSemaphoreGive(lock);
  esp_timer_start_once(timer1_handle, 8000000); // 8 seconds
}
// Callback function for TIMER1 when Door is release (auto closer work now)
void timer1_callback(void *arg)
{
  Serial.println("                                        TIMER1 triggered  8s  MOTOR is turning off ");
  xSemaphoreTake(lock, portMAX_DELAY);
  doorMode = 0;
  xSemaphoreGive(lock);
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
  while (1)
  {
    uint32_t ulNotificationValue;
    Freq = getCpuFrequencyMhz();
    // Wait for a notification from the timer
    ulNotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

    if (ulNotificationValue != 0)
    {
      // A notification was received, process it

      reset_mode = !reset_mode;
      if (reset_mode)
      {
        vTaskSuspend(dc_motor_Handle); // Suspend the task
        // vTaskSuspend(pir_sensor_Handle);
        doorMode = 0;
        esp_timer_stop(timer0_handle);
        esp_timer_stop(timer1_handle);
      }
      else
      {
        xTaskResumeFromISR(dc_motor_Handle); // Resume the task from ISR
        // xTaskResumeFromISR(pir_sensor_Handle); // Resume the task from ISR
        doorMode = 0;
      }
    }
    else
    {
      // A timeout occurred
      if (reset_mode)
      {
        Serial.println("OPENING");
        // Serial.println(Freq);
      }
      else
      {
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

// Task L (low priority)
void doTaskL(void *parameters)
{

  TickType_t timestamp;

  // Do forever
  while (1)
  {
    xSemaphoreTake(lock, portMAX_DELAY);
    // Serial.print("                                      Task Sensor is on");
    // do something over here
    if (digitalRead(PIR_SENSOR_PIN) == HIGH && doorMode == 1)
    {
      doorMode = 0;
    }
    else if (digitalRead(PIR_SENSOR_PIN) == LOW && doorMode == 1)
    {
      sensorMode = 1;
      doorMode = 2;
    }
    else if (digitalRead(PIR_SENSOR_PIN) == HIGH && doorMode == 3)
    {
      // reset timer 1 if sensor is activate when door is on hold
      sensorMode = 1;
      Serial.print("                                          RESET timer 0");
      esp_timer_stop(timer0_handle);
      esp_timer_start_once(timer0_handle, 8000000); // 1/32 seconds
    }
    else if (digitalRead(PIR_SENSOR_PIN) == HIGH && doorMode == 4)
    {
    }
    else
    {
      sensorMode = 0;
    }
    xSemaphoreGive(lock);

    // Go to sleep
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

// Task M (medium priority)
void doTaskM(void *parameters)
{
  while (1)
  {

    // digitalWrite(RELAY_INPUT,HIGH);
    // vTaskDelay(500 / portTICK_PERIOD_MS);
    // digitalWrite(RELAY_INPUT,LOW);
    // vTaskDelay(500 / portTICK_PERIOD_MS);

    xSemaphoreTake(lock, portMAX_DELAY);

    if (doorMode == 2 || doorMode == 3)
    {
      digitalWrite(RELAY_INPUT, HIGH);
    }
    else if (doorMode == 4 && digitalRead(PIR_SENSOR_PIN) == HIGH)
    {
      digitalWrite(RELAY_INPUT, HIGH);
    }
    else
    {
      digitalWrite(RELAY_INPUT, LOW);
    }

    xSemaphoreGive(lock);

    if (sensorMode == 1)
    {
      // Serial.print("Task LED detect a sensor >>>>>>> turn the LED on");
      digitalWrite(LED_PIN, HIGH);
    }
    else if (sensorMode == 0)
    {
      digitalWrite(LED_PIN, LOW);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Task H (high priority)
void doTaskH(void *parameters)
{
  while (1)
  {
    read_analog();
    xSemaphoreTake(lock, portMAX_DELAY);
    // MOTOR DOING SOME WORK HERE
    if (doorMode == 2)
    {
      Serial.println("                            DoorMode = 2");
      open_door(stepsPerRevolution);
    }
    else if (doorMode == 10){
      open_door(stepAlreadyOpen);
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

void check_open_distance()
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  int distanceCM;
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED / 2;
  if (distanceCM <= min_distance)
  {
    Serial.println(distanceCM);
    Serial.print("End reached");
  }
}
//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup()
{
  // Configure Serial
  Serial.begin(921600);

  // Set up range detector
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  initialize_timer();
  // GPIO PIN
  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  ledcSetup(CHANNEL, 2000, 12);
  // Create the fire alarm timer

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(DRIVER_DIR, OUTPUT);
  pinMode(DRIVER_PUL, OUTPUT);

  // relay input
  pinMode(RELAY_INPUT, OUTPUT);
  digitalWrite(RELAY_INPUT, LOW);

  attachInterrupt(digitalPinToInterrupt(OPEN_DOOR_BUTTON_PIN), openning_door_signal, FALLING);
  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Create semaphores and mutexes before starting tasks
  lock = xSemaphoreCreateMutex();

  // The order of starting the tasks matters to force priority inversion

  // Start Button Task (low priority)
  xTaskCreatePinnedToCore(reset_trigger_task, "reset_trigger_Task", 2048, NULL, configMAX_PRIORITIES - 1, &reset_trigger_Handle, app_cpu);

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

void loop()
{
  // Execution should never get here
}
