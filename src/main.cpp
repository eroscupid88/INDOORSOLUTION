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


Servo myservo; // create servo object to control a servo
int posVal = 0; // variable to store the servo position
int servoPin = 15; // Servo motor pin

// motor
int in1Pin = 12; // Define L293D channel 1 pin
int in2Pin = 14; // Define L293D channel 2 pin
int enable1Pin = 27; // Define L293D enable 1 pin
int channel = 0;

// define task
static TaskHandle_t task1 = NULL;
static TaskHandle_t task2 = NULL;

boolean rotationDir; // Define a variable to save the motor's rotation direction
int rotationSpeed; // Define a variable to save the motor rotation speed

#define PIN_LED 2
#define PIN_BUTTON 13

const char msg[] = "GE495 capstone project running demo";
void startTask1 (void *parameter){
  Serial.println("Button Task has been started");
  while (1)
  {
    if (digitalRead(PIN_BUTTON) == LOW)
    {
      Serial.println("button was pressed");
      digitalWrite(PIN_LED, HIGH);
    }
    else
    {
      digitalWrite(PIN_LED, LOW);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}
void startTask2 (void *parameter){

    while (1){
      Serial.print("*");
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void driveMotor(boolean dir, int spd) {
if (dir) {// Control motor rotation direction
digitalWrite(in1Pin, HIGH);
digitalWrite(in2Pin, LOW);
}
else {
digitalWrite(in1Pin, LOW);
digitalWrite(in2Pin, HIGH);
}
ledcWrite(channel, spd); // Control motor rotation speed
}

void setup() {
  Serial.begin(921600);
  Serial.println("FREE RTOS Start");
  myservo.setPeriodHertz(50); // standard 50 hz servo
  myservo.attach(servoPin, 500, 2500); // attaches the servo on servoPin to the servo
  // Initialize the pin into an output mode:
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  ledcSetup(channel,1000,11); //Set PWM to 11 bits, range is 0-2047
  ledcAttachPin(enable1Pin,channel);


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
  Serial.println("Start");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  // for (posVal = 0; posVal <= 90; posVal += 1) { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   myservo.write(posVal); // tell servo to go to position in variable 'pos'  
  // delay(15); // waits 15ms for the servo to reach the position
  // }
  // for (posVal = 90; posVal >= 0; posVal -= 1) { // goes from 180 degrees to 0 degrees
  //   myservo.write(posVal); // tell servo to go to position in variable 'pos'
  //   delay(15); // waits 15ms for the servo to reach the position
  // }

  // int potenVal = analogRead(A0);// Convert the voltage of rotary potentiometer into digital
  // rotationSpeed = potenVal - 2048;
  // if (potenVal > 2048)
  // rotationDir = true;
  // else
  // rotationDir = false;
  // // Calculate the motor speed
  // rotationSpeed = abs(potenVal - 2048);
  // //Control the steering and speed of the motor
  // driveMotor(rotationDir, constrain(rotationSpeed,0,2048));
}







