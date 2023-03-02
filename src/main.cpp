#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ESP32Servo.h>
#include "esp_timer.h"

// 2 Define all function
void timer0_callback(void* arg);
void timer1_callback(void* arg);
void led_strigger_task(void *pvParmeter);
void reset_button_task(void *pvParameter);
void pir_sensor_task(void *pvParameter);
void motor_task(void *pvParmeter);

// Define the timer handle
esp_timer_handle_t timer0_handle;
esp_timer_handle_t timer1_handle;

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
#define BUTTON_PRESS_DELAY_MS 2000 //Length of button press
// Define a global variable to hold the button state
volatile bool suspended = false;
volatile int sensorMode = 0; //variable to determine if the IR sensor is high or not
// doorMode 0: door is shut
// doorMode 1: door is in hold position
// doormode 2: door is in shutting down position
volatile int doorMode = 0;
volatile int a = 0;

SemaphoreHandle_t sensorMutex; // mutex to control access to sensor output
SemaphoreHandle_t motorMutex;

TaskHandle_t pir_sensor_Handle;
TaskHandle_t led_strigger_Handle;
TaskHandle_t dc_motor_Handle;
TaskHandle_t reset_button_Handle;

TimerHandle_t Fire_alarm_TimerHandle = NULL;
/*******************************HELPER FUNCTIONS*******************************************************/
void run_motor(int number){
    // Serial.println("motor is running ********");
    // Serial.println(number);
    ledcWrite(CHANNEL, number);
}

void buttonInterrupt(){
  if (digitalRead(RESET_BUTTON_PIN) == LOW)
  {
    // Start the timer
    xTimerStart(Fire_alarm_TimerHandle, 0);
  }
  else
  {
    // Button released before 2 seconds
    xTimerStop(Fire_alarm_TimerHandle, 0);
  }
}

void reset_button_TimerCallback(TimerHandle_t xTimer){
  // Stop the timer and notify the long press task
  xTimerStop(Fire_alarm_TimerHandle, 0);
  xTaskNotifyGive(reset_button_Handle);
}

/************************************TIMERS************************************************************/
// Callback function for TIMER0 when Door is in wide open 
void timer0_callback(void* arg) {
    Serial.println("TIMER0 triggered");
    // Configure and start TIMER1
    doorMode = 1;
    // activate timer 1 to start hold position(force is equal to auto closer)
    esp_timer_create_args_t timer1_args = {.callback = timer1_callback,.name = "timer1"};
    esp_timer_create(&timer1_args, &timer1_handle);
    esp_timer_start_once(timer1_handle, 6000000);  // 6 seconds
    vTaskResume(pir_sensor_Handle);
    // a = 1;
    // esp_timer_stop(timer0_handle);
}
// Callback function for TIMER1 when Door is release (auto closer work now)
void timer1_callback(void* arg) {
    Serial.println("TIMER1 triggered");
    // esp_timer_stop(timer1_handle);
    doorMode = 2;
    sensorMode = 0;
    digitalWrite(PIN_R_PWM_EN, LOW);
    digitalWrite(PIN_L_PWM_EN,LOW);
    // vTaskSuspend(dc_motor_Handle);
    // vTaskResume(pir_sensor_Handle);
}

/*************************************TASKS**************************************************************/
void led_strigger_task(void *pvParmeter){
    while(1){
        if(sensorMode == 1){
            // Serial.println("LED TRIGGER<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>");
            digitalWrite(LED_PIN,HIGH);
        }
        else if(sensorMode ==0){
            digitalWrite(LED_PIN,LOW);
        }
        Serial.println(sensorMode);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        // vTaskResume(pir_sensor_Handle);
        // vTaskSuspend(NULL);
    }
}

void reset_button_task(void *pvParameter){
    uint32_t notificationValue;

    while (1) {
        notificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(3000)); //timeout after half a second

        if (notificationValue > 0) {
        // handle notification
            vTaskSuspend(dc_motor_Handle);
            vTaskSuspend(led_strigger_Handle);
            vTaskSuspend(pir_sensor_Handle);
            Serial.println("Button pressed for more than 2 seconds!");
        } 
        // else if{;}
        
        else {
        // notification timed out
            Serial.println("Reset button not pressed long enough");
            vTaskResume(pir_sensor_Handle);
            vTaskSuspend(NULL);
        }
    }

            
            
      
    
    return;
}
void pir_sensor_task(void *pvParameter) {
    esp_timer_create_args_t timer0_args = {.callback = timer0_callback,.name = "timer0"};
    esp_timer_create(&timer0_args, &timer0_handle);
    
    // esp_timer_start_periodic(timer0_handle, 10000000/2);  // 5 seconds
    
    while (1)
    {
        // Serial.println("Entering PIR SENSOR TASK");
        if (digitalRead(PIR_SENSOR_PIN) == HIGH && doorMode == 0) {
            // Serial.println("Sensor Detected************************************");
            sensorMode = 1;
            vTaskDelay(100/portTICK_PERIOD_MS);
            // if (a == 1){
            esp_timer_start_once(timer0_handle, 10000000/2);  // 5 seconds
            // }
            vTaskResume(dc_motor_Handle);
            vTaskSuspend(NULL);

        } 
        else if  (digitalRead(PIR_SENSOR_PIN) == HIGH && doorMode == 1){
            // in door Hold position
            sensorMode = 1;
            esp_timer_stop(timer1_handle);
            esp_timer_start_once(timer1_handle, 6000000);
            }
        else if (doorMode == 2){
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            sensorMode = 0;
            doorMode = 0;
        }
        else {
            sensorMode = 0;
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void motor_task(void *pvParmeter){
    Serial.println("Entering Motor Task ");
    pinMode(PIN_R_PWM_EN, OUTPUT);
    pinMode(PIN_L_PWM_EN, OUTPUT);
    //PWM signal control but Potentiometer
    ledcSetup(CHANNEL, 1000, 12);
    ledcAttachPin(PIN_RPWM, CHANNEL);
    
    while (1){

            if (doorMode == 0 && sensorMode == 1){
                digitalWrite(PIN_R_PWM_EN, HIGH);
                digitalWrite(PIN_L_PWM_EN,HIGH);
                int adcVal = analogRead(PIN_ANALOG_IN); // read adc
                int pwmVal = adcVal;
                // Serial.println(pwmVal);
                run_motor(pwmVal);
                // Configure and start TIMER0
            }
            else if (doorMode == 1 && sensorMode == 1) {
                digitalWrite(PIN_R_PWM_EN,HIGH);
                digitalWrite(PIN_L_PWM_EN,HIGH);
                run_motor(1000); // Apply force enough to hold the door TBD later
            }
            else if (doorMode == 2) {
                // is closing but received sensor signal then open again. for now let motor shut down
                digitalWrite(PIN_R_PWM_EN,LOW);
                digitalWrite(PIN_L_PWM_EN,LOW);
            }
            else {
                digitalWrite(PIN_R_PWM_EN,LOW);
                digitalWrite(PIN_L_PWM_EN,LOW);
                // doorMode = 0;
            }
            Serial.print("                      sensor mode is: ");
            Serial.println(sensorMode);
            Serial.print("                      Door mode is: ");
            Serial.println(doorMode);
        vTaskDelay(100 / portTICK_PERIOD_MS);  
    }
}


/**************************************SETUP**********************************************************/

void setup() {
    Serial.begin(115200);
    pinMode(PIR_SENSOR_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(PIN_LPWM, OUTPUT);
    pinMode(PIN_RPWM, OUTPUT);
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  

    // Create the fire alarm timer 
    Fire_alarm_TimerHandle = xTimerCreate("fire_alarm_timer", BUTTON_PRESS_DELAY_MS / portTICK_PERIOD_MS, pdFALSE, 0, reset_button_TimerCallback);
  


    xTaskCreate(reset_button_task, "reset_button_task", 2048, NULL, 1, &reset_button_Handle);
    // Attach the reset button interrupt
    attachInterrupt(digitalPinToInterrupt(RESET_BUTTON_PIN), buttonInterrupt, CHANGE);

    
    // xTaskCreate(pir_sensor_task, "pir_sensor_task", 4096, NULL, 2, &pir_sensor_Handle);
    // xTaskCreate(led_strigger_task, "led_trigger_task", 4096, NULL, 10, &led_strigger_Handle);
    // xTaskCreate(motor_task, "motor_task", 4096, NULL, 3, &dc_motor_Handle);
    
    // xTaskCreate(dc_motor_task, "dc_motor_task", 4096, NULL, 3, &dc_motor_Handle);
    // xTaskCreate(motor_task, "motor_task", 4096, NULL, 3,&dc_motor_Handle);
    

    vTaskStartScheduler();
    
    vTaskSuspend(dc_motor_Handle);
}

void loop() {
}
