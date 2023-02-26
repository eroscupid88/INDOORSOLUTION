#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ESP32Servo.h>

// #include <ledc.h>


#define PIR_SENSOR_PIN 4  // infrared sensor pin
#define LED_PIN 2   // led pin
#define IN_1_PIN 12 // L293D channel 1 pin
#define IN_2_PIN 14 // L293D channel 2 pin
#define EN_1_PIN 13 // L293D enable 1 pin
#define FRQ 1000    // PWM frequency
#define PWM_BIT 11   // PWM Precision
#define PIN_ANALOG_IN 15  // Potentiometer
#define PIN_R_PWM_EN 5
#define PIN_L_PWM_EN 18
#define CHANNEL 0
#define PIN_LPWM 19
#define PIN_RPWM 21

boolean rotationDir; // boolean variable to save the motor's rotation direction
boolean motorfirstRun; //boolean variable to control motor runtime to 5 seconds
int rotationSpeed; // variable to save the motor rotation speed
int sensorMode = 0; //variable to determine if the IR sensor is high or not
unsigned long startTime; // variable to track the start time of the initial sensor trigger
unsigned long currentTime; // variable to track the current time 

SemaphoreHandle_t sensorMutex; // mutex to control access to sensor output
SemaphoreHandle_t motorMutex;
TaskHandle_t pir_sensor_Handle;
TaskHandle_t led_strigger_Handle;
TaskHandle_t dc_motor_Handle;

void led_strigger_task(void *pvParmeter){
    
    while(1){
        Serial.println("Entering LED TRIGGER<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>");
        if(xSemaphoreTake(sensorMutex,portMAX_DELAY)==pdTRUE){
            if(sensorMode == 1){
                digitalWrite(LED_PIN,HIGH);
            }
            else if(sensorMode ==0){
                digitalWrite(LED_PIN,LOW);
            }
            Serial.println(sensorMode);
            xSemaphoreGive(sensorMutex);
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
        vTaskResume(pir_sensor_Handle);
        // vTaskSuspend(NULL);
    }

}

/*Helper Functions*/

void run_motor(int number){
    // Serial.println("motor is running ********");
    // Serial.println(number);
    ledcWrite(CHANNEL, number);
}

void pir_sensor_task(void *pvParameter) {
    Serial.println("Entering PIR SENSOR TASK");
    while (1)
    {
        if (digitalRead(PIR_SENSOR_PIN) == HIGH) {
            Serial.println("Sensor Detected>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
            if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE)
            {
                sensorMode = 1;
                xSemaphoreGive(sensorMutex);
            }
        } else {
            if(xSemaphoreTake(sensorMutex,portMAX_DELAY)==pdTRUE){
                sensorMode = 0;
                xSemaphoreGive(sensorMutex);
            }
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
        vTaskSuspend(NULL);
    }
}

void motor_task(void *pvParmeter){
    Serial.println("Entering Motor Task ");
    pinMode(PIN_LPWM, OUTPUT);
    pinMode(PIN_RPWM, OUTPUT);
    //PWM signal control but Potentiometer
    ledcSetup(CHANNEL, 1000, 12);
    ledcAttachPin(PIN_RPWM, CHANNEL);
    while (1){
        // Serial.println("Potentiometer Task to set up PWM");
        // Serial.println("Received Semaphore");
        // Serial.println(sensorMode);
        int adcVal = analogRead(PIN_ANALOG_IN); // read adc
        int pwmVal = adcVal;
        // Serial.println(pwmVal);
        run_motor(pwmVal);
            // xSemaphoreGive(sensorMutex);
        
        // Timer Activate to suspense the Motor Task
        // Todo

        // vTaskDelay(1000/portTICK_PERIOD_MS);
        // vTaskResume(pir_sensor_Handle);
        
        
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PIR_SENSOR_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    sensorMutex = xSemaphoreCreateMutex();
    xTaskCreate(pir_sensor_task, "pir_sensor_task", 4096, NULL, 1, &pir_sensor_Handle);
    xTaskCreate(led_strigger_task, "led_trigger_task", 4096, NULL, 2, &led_strigger_Handle);
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 3, &dc_motor_Handle);
    
    // xTaskCreate(dc_motor_task, "dc_motor_task", 4096, NULL, 3, &dc_motor_Handle);
    // xTaskCreate(motor_task, "motor_task", 4096, NULL, 3,&dc_motor_Handle);
    

    vTaskStartScheduler();
}

void loop() {
    // Empty loop
}
