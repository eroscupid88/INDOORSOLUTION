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
#define CHN 0   // PWM channel
#define FRQ 1000    // PWM frequency
#define PWM_BIT 11   // PWM Precision

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


void drive_motor(boolean dir, int spd);

void control_tasks(void *pvParameter){

}

void pir_sensor_task(void *pvParameter) {
    pinMode(PIR_SENSOR_PIN, INPUT);
    while (1){
        Serial.println("Entering PIR SENSOR TASK");

        if (digitalRead(PIR_SENSOR_PIN) == HIGH) {
            if(xSemaphoreTake(sensorMutex,portMAX_DELAY)==pdTRUE){
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
        vTaskResume(led_strigger_Handle);
        vTaskSuspend(NULL);
    }

}

void dc_motor_task(void *pvParameter){
    pinMode(IN_1_PIN, OUTPUT);
    pinMode(IN_2_PIN,OUTPUT);
    pinMode(EN_1_PIN, OUTPUT);
    analogWriteFrequency(FRQ);   // USE LEDC(We will for the big motor) to modify PWM precision /analog has precision pwm from 0-255
    
    // ledcSetup(CHN, FRQ, PWM_BIT);  // Set the PWM frequency to 20 kHz, with 8-bit resolution
    // ledcAttachPin(EN_1_PIN, CHN);

    while(1){
        Serial.println("Entering DC MOTOR TASK");
        int potenVal = analogRead(A0); //Convert the voltage of rotary potentiometer into digital
        Serial.println(potenVal);
        rotationSpeed = potenVal - 2048;
        if (potenVal >2048) rotationDir = true;
        else rotationDir = false;

        //Control the motor speed; I divided by 8 because the pwm goes up to 255 but with LEDC you shouldnt
        rotationSpeed = abs(potenVal - 2048)/8;
        if (rotationSpeed > 255) rotationSpeed = 255; // same for this one; remove when using ledc

        //Control the steering and speed of the motor
        
        if(xSemaphoreTake(sensorMutex,portMAX_DELAY)==pdTRUE){ // test: run the motor if sensor is on
            if (sensorMode) drive_motor(rotationDir, constrain(rotationSpeed,0,255));
            else drive_motor(rotationDir,0);
            xSemaphoreGive(sensorMutex);


        }
        vTaskDelay(100/portTICK_PERIOD_MS);

        vTaskResume(pir_sensor_Handle);
    }

}

void led_strigger_task(void *pvParmeter){
    pinMode(LED_PIN, OUTPUT);
    while(1){
        Serial.println("Entering LED TRIGGER");
        if(xSemaphoreTake(sensorMutex,portMAX_DELAY)==pdTRUE){
            if(sensorMode == 1){
                digitalWrite(LED_PIN,HIGH);
                
            }
            else if(sensorMode ==0){
                digitalWrite(LED_PIN,LOW);
            }
            xSemaphoreGive(sensorMutex);
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
        vTaskResume(dc_motor_Handle);
        vTaskSuspend(NULL);
    }

}


/*Helper Functions*/

void drive_motor(boolean dir, int spd){
    if(dir){ //Control motor rotation direction
        digitalWrite(IN_1_PIN,HIGH);
        digitalWrite(IN_2_PIN, LOW);
    }
    else{
        digitalWrite(IN_1_PIN,LOW);
        digitalWrite(IN_2_PIN, HIGH);      
    }
    analogWrite(EN_1_PIN,spd); // Control motor rotation speed

}


void setup() {
    Serial.begin(115200);
    sensorMutex = xSemaphoreCreateMutex();
    xTaskCreate(pir_sensor_task, "pir_sensor_task", 4096, NULL, 1, &pir_sensor_Handle);
    xTaskCreate(dc_motor_task, "dc_motor_task", 4096, NULL, 3, &dc_motor_Handle);
    xTaskCreate(led_strigger_task, "led_trigger_task", 4096, NULL, 2, &led_strigger_Handle);
    vTaskStartScheduler();
}

void loop() {
    // Empty loop
}
