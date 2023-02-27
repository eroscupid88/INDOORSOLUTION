#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ESP32Servo.h>
#include "esp_timer.h"

// 2 Timers callback function
void timer0_callback(void* arg);
void timer1_callback(void* arg);


// Define the timer handle
esp_timer_handle_t timer0_handle;
esp_timer_handle_t timer1_handle;

#define PIR_SENSOR_PIN 4  // infrared sensor pin
#define LED_PIN 2   // led pin
#define IN_1_PIN 12 // L293D channel 1 pin
#define IN_2_PIN 14 // L293D channel 2 pin
#define EN_1_PIN 13 // L293D enable 1 pin
#define FRQ 1000    // PWM frequency
#define PWM_BIT 11   // PWM Precision
#define PIN_ANALOG_IN 15  // Potentiometer
#define PIN_R_PWM_EN 32
#define PIN_L_PWM_EN 33
#define CHANNEL 0
#define PIN_LPWM 19
#define PIN_RPWM 21

int sensorMode = 0; //variable to determine if the IR sensor is high or not
int doorMode = 0;
int a = 0;
SemaphoreHandle_t sensorMutex; // mutex to control access to sensor output
SemaphoreHandle_t motorMutex;
TaskHandle_t pir_sensor_Handle;
TaskHandle_t led_strigger_Handle;
TaskHandle_t dc_motor_Handle;

void led_strigger_task(void *pvParmeter){
    
    while(1){
        if(sensorMode == 1){
            Serial.println("LED TRIGGER<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>");
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

/*Helper Functions*/

void run_motor(int number){
    // Serial.println("motor is running ********");
    Serial.println(number);
    ledcWrite(CHANNEL, number);
}

void pir_sensor_task(void *pvParameter) {
    
    while (1)
    {
        Serial.println("Entering PIR SENSOR TASK");
        
        if (digitalRead(PIR_SENSOR_PIN) == HIGH) {
            Serial.println("Sensor Detected************************************");
            sensorMode = 1;
            vTaskDelay(100/portTICK_PERIOD_MS);
            if (a == 1){
                esp_timer_start_periodic(timer0_handle, 10000000/2);  // 5 seconds
        }
            vTaskResume(dc_motor_Handle);
            vTaskSuspend(NULL);
        } else {
            sensorMode = 0;
            vTaskDelay(100/portTICK_PERIOD_MS);
        }
    }
}

// Callback function for TIMER0
void timer0_callback(void* arg) {
    Serial.println("TIMER0 triggered");
    // Configure and start TIMER1
    // if(xSemaphoreTake(motorMutex,portMAX_DELAY)==pdTRUE){
    doorMode = 1;
    esp_timer_create_args_t timer1_args = {.callback = timer1_callback,.name = "timer1"};
    esp_timer_create(&timer1_args, &timer1_handle);
    esp_timer_start_periodic(timer1_handle, 6000000);  // 8 second
        // xSemaphoreGive(motorMutex);
    // }
    a = 1;
    esp_timer_stop(timer0_handle);
}
void timer1_callback(void* arg) {
    Serial.println("TIMER1 triggered");
    esp_timer_stop(timer1_handle);
    doorMode = 0;
    sensorMode = 0;
    digitalWrite(PIN_R_PWM_EN,LOW);
    digitalWrite(PIN_L_PWM_EN,LOW);
    vTaskSuspend(dc_motor_Handle);
    vTaskResume(pir_sensor_Handle);
}

void motor_task(void *pvParmeter){
    Serial.println("Entering Motor Task ");
    pinMode(PIN_R_PWM_EN, OUTPUT);
    pinMode(PIN_L_PWM_EN, OUTPUT);
    //PWM signal control but Potentiometer
    ledcSetup(CHANNEL, 1000, 12);
    ledcAttachPin(PIN_RPWM, CHANNEL);
    esp_timer_create_args_t timer0_args = {.callback = timer0_callback,.name = "timer0"};
    esp_timer_create(&timer0_args, &timer0_handle);
    esp_timer_start_periodic(timer0_handle, 10000000/2);  // 5 seconds
    while (1){
        // if(xSemaphoreTake(motorMutex,portMAX_DELAY)==pdTRUE){
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
                run_motor(500);
            }
            else {
                digitalWrite(PIN_R_PWM_EN,LOW);
                digitalWrite(PIN_L_PWM_EN,LOW);
                doorMode = 0;
            }
            Serial.print("sensor mode is: ");
            Serial.println(sensorMode);
            Serial.print("Door mode is: ");
            Serial.println(doorMode);
            // xSemaphoreGive(motorMutex);
        // }
        vTaskDelay(100 / portTICK_PERIOD_MS);
        // vTaskResume(pir_sensor_Handle);
        
        
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PIR_SENSOR_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(PIN_LPWM, OUTPUT);
    pinMode(PIN_RPWM, OUTPUT);
    
    xTaskCreate(pir_sensor_task, "pir_sensor_task", 4096, NULL, 1, &pir_sensor_Handle);
    xTaskCreate(led_strigger_task, "led_trigger_task", 4096, NULL, 10, &led_strigger_Handle);
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 3, &dc_motor_Handle);
    
    // xTaskCreate(dc_motor_task, "dc_motor_task", 4096, NULL, 3, &dc_motor_Handle);
    // xTaskCreate(motor_task, "motor_task", 4096, NULL, 3,&dc_motor_Handle);
    

    vTaskStartScheduler();
    vTaskSuspend(dc_motor_Handle);
}

void loop() {
    // Empty loop
}
