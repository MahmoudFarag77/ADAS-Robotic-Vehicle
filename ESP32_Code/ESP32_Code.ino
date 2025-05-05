#include <Arduino.h>              // Standard Arduino Library
#include <freertos/FreeRTOS.h>    // FreeRTOS Main Library
#include <freertos/task.h>        // FreeRTOS Tasks Library
#include "BluetoothSerial.h"      // Bluetooth library
#include <ESP32Servo.h>           // Servo Motor library
#include <Wire.h>                 // I2C Protocol library

// Bluetooth Serial
BluetoothSerial serialBT;
// Queue to hold received I2C signals
QueueHandle_t signalQueue;

// --- Pins Delcaration ---
// L298N H-Bridge (DC Motor Driver)
#define EN_A 32
#define IN_1 33
#define IN_2 25
#define IN_3 26
#define IN_4 27
#define EN_B 14
// SG90 Servo (Steering Servo)
Servo SteeringServo;
#define SteeringPin 18
// Front Ultrasonic Sensor (AEB and ACC)
#define FRONT_ULTRASONIC_TRIGGER_PIN 3
#define FRONT_ULTRASONIC_ECHO_PIN 34
// Blinkers LEDs (Blinkers and Brake LEDs)
#define BACK_STOP_LED 23
#define LEFT_LED_BLINKER 19
#define RIGHT_LED_BLINKER 5
// IR Sensors (LKA)
#define LEFT_IR 35
#define RIGHT_IR 39
// Left Ultrasonic Sensor and LED (BSW)
#define ULTRASONIC_TRIGGER_LEFT_PIN 15
#define ULTRASONIC_ECHO_LEFT_PIN 2
#define LEFT_BSW_LED 16
// Right Ultrasonic Sensor and LED (BSW)
#define ULTRASONIC_TRIGGER_RIGHT_PIN 0
#define ULTRASONIC_ECHO_RIGHT_PIN 4
#define RIGHT_BSW_LED 17
// LDR and Front LEDs (ALS)
#define ANALOG_LDR_PIN 36
#define HIGH_BEAM_LED 13
#define LOW_BEAM_LED 12
// Define I2C address
#define I2C_ADDRESS 0x08

// --- Macro Values --- 
// AEB Macros
#define AEB_Threshold 25
// ACC Macros
#define ACC_Decrease_Threshold 50
#define ACC_Increase_Threshold 80
// TSDR Macros 
#define SPEED_LIMIT_60 180        // Speed Limit = 180 PWM
#define SPEED_LIMIT_120 220       // Speed Limit = 220 PWM
#define STOP_Time 5000            // Time to STOP before moving again (STOP Sign)
#define NO_SPEED_LIMIT 255        // No Speed Limit
// LKA Macros
#define STRAIGHT 90               // Servo angle 90
#define LEFT 60                   // Servo angle 60
#define LKA_LEFT 75               // Servo angle 75
#define RIGHT 120                 // Servo angle 120
#define LKA_RIGHT 105             // Servo angle 105
// BSW Macros
#define BSW_Distance 20
// ALS Macros
#define DAY_RUNNING_LED 20
#define ALS_Threshold_1 750
#define ALS_Threshold_2 2500
#define ALS_Threshold_3 3500

// --- Global Variables ---
// Bluetooth Connection Variables
char btSignal;                    // char to recieve bluetooth signal
char Recall_Signal;               // char for recalling recieved bluetooth signal
char Last_BT_STOP;                // char for recalling last recieved bluetooth signal after detecting STOP sign (TSDR)
// Speed Variables
int SPEED = 130;                  // Speed value (PWM) (Initially = 130 PWM)
int OLD_SPEED;                    // To stop printing same speed value
int MAX_SPEED = 255;              // Maximum speed value (PWM) (TSDR)   -- Initially No Speed Limit
// Blinkers Variables
int BLINKER_VALUE = 1;            // Blinker LEDs initial value
int LEFT_BLINKER_STATE = 0;       // Left sign LED Activate/Deactivate  -- Initially OFF
int RIGHT_BLINKER_STATE = 0;      // Right sign LED Activate/Deactivate -- Initially OFF
// AEB Variables
int AEB_Counter = 0;              // AEB distance counter
// LKA Variables
int LKA_Counter = 0;              // LKA Error counter
// BSW Variables
int LBSW_LED_VALUE = 1;           // LBSW LED initial value
int RBSW_LED_VALUE = 1;           // RBSW LED initial value
// ADAS Features Variables 
int ACC_STATE = 0;                // ACC Function Activate/Deactivate   -- Initially OFF
int TSDR_STATE = 0;               // TSDR Function Activate/Deactivate  -- Initially OFF
int LKA_STATE = 0;                // LKA Function Activate/Deactivate   -- Initially OFF
int ALS_STATE = 0;                // ALS Function Activate/Deactivate   -- Initially OFF

// Moving forward function (Anti-clockwise)
void Forward(){
  digitalWrite(IN_1,LOW);
  digitalWrite(IN_2,HIGH);
  digitalWrite(IN_3,HIGH);
  digitalWrite(IN_4,LOW);
  analogWrite(EN_A,SPEED);
  analogWrite(EN_B,SPEED);
  // Turn off brake LED
  digitalWrite(BACK_STOP_LED, LOW);
}

// Moving backward function (Clockwisse)
void Backward(){
  digitalWrite(IN_1,HIGH);
  digitalWrite(IN_2,LOW);
  digitalWrite(IN_3,LOW);
  digitalWrite(IN_4,HIGH);
  analogWrite(EN_A,SPEED);
  analogWrite(EN_B,SPEED);
  // Turn off brake LED
  digitalWrite(BACK_STOP_LED, LOW);
}

// Stop function 
void Stop(){
  digitalWrite(IN_1,LOW);
  digitalWrite(IN_2,LOW);
  digitalWrite(IN_3,LOW);
  digitalWrite(IN_4,LOW);
  analogWrite(EN_A,SPEED);
  analogWrite(EN_B,SPEED);
  // Reset the initial speed value
  SPEED = 130;
  // Turn ON brake LED
  digitalWrite(BACK_STOP_LED, HIGH);
}

// Bluetooth recieve FreeRTOS Function
void BT_Receive(void *pvParameters){
  while(1){
    while(serialBT.available()){
      btSignal = serialBT.read();
      // Move Forward
      if(btSignal == 'f'){
        Recall_Signal = 'f';
        LEFT_BLINKER_STATE = 0;
        RIGHT_BLINKER_STATE = 0;
        serialBT.print("Speed = ");
        serialBT.println(SPEED);
        AEB_Counter = 0;
        // Force STOP ACC
        if(ACC_STATE == 1){
          ACC_STATE = 0;
          serialBT.println("ACC OFF");
        }
      }
      // Move Backward
      if(btSignal == 'b'){
        Recall_Signal = 'b';
        LEFT_BLINKER_STATE = 0;
        RIGHT_BLINKER_STATE = 0;
        serialBT.print("Speed = ");
        serialBT.println(SPEED);
        AEB_Counter = 0;
        // Force STOP ACC
        if(ACC_STATE == 1){
          ACC_STATE = 0;
          serialBT.println("ACC OFF");
        }
      }
      // Steer Left
      if(btSignal == 'l'){
        SteeringServo.write(LEFT);
        LEFT_BLINKER_STATE = 1;
        RIGHT_BLINKER_STATE = 0;
        // Force STOP LKA
        if(LKA_STATE == 1){
          LKA_STATE = 0;
          serialBT.println("LKA OFF");
        }
      }
      // Steer Right
      if(btSignal == 'r'){
        SteeringServo.write(RIGHT);
        LEFT_BLINKER_STATE = 0;
        RIGHT_BLINKER_STATE = 1;
        // Force STOP LKA
        if(LKA_STATE == 1){
          LKA_STATE = 0;
          serialBT.println("LKA OFF");
        }
      }
      // Steer Straight
      if(btSignal == 'n'){
        SteeringServo.write(STRAIGHT);
        LEFT_BLINKER_STATE = 0;
        RIGHT_BLINKER_STATE = 0;
        // Force STOP LKA
        if(LKA_STATE == 1){
          LKA_STATE = 0;
          serialBT.println("LKA OFF");
        }
      }
      // STOP
      if(btSignal == 's'){
        Recall_Signal = 's';
        SteeringServo.write(STRAIGHT);
        LEFT_BLINKER_STATE = 0;
        RIGHT_BLINKER_STATE = 0;
        // Force STOP ACC
        if(ACC_STATE == 1){
          ACC_STATE = 0;
          serialBT.println("ACC OFF");
        }
        // Force STOP LKA
        if(LKA_STATE == 1){
          LKA_STATE = 0;
          serialBT.println("LKA OFF");
        }
      }
      // Turn ON Hazard LEDs
      if(btSignal == 'h'){  
        LEFT_BLINKER_STATE = 1;
        RIGHT_BLINKER_STATE = 1;
      }
      // Turn OFF Hazard LEDs
      if(btSignal == 'k'){  
        LEFT_BLINKER_STATE = 0;
        RIGHT_BLINKER_STATE = 0;
      }
      // Increase Speed Manually
      if(btSignal == 'i'){
        if(SPEED < MAX_SPEED){
          OLD_SPEED = SPEED;
          SPEED = OLD_SPEED + 5;
          if(OLD_SPEED != SPEED){
            serialBT.print("Speed = ");
            serialBT.println(SPEED);
          }
        }
        // Force STOP ACC
        if(ACC_STATE == 1){
          ACC_STATE = 0;
          serialBT.println("ACC OFF");
        }
      }
      // Decrease Speed Manually
      if(btSignal == 'd'){
        if(SPEED > 130){
          OLD_SPEED = SPEED;
          SPEED = OLD_SPEED - 5;
          if(OLD_SPEED != SPEED){
            serialBT.print("Speed = ");
            serialBT.println(SPEED);
          }
        }
        // Force STOP ACC
        if(ACC_STATE == 1){
          ACC_STATE = 0;
          serialBT.println("ACC OFF");
        }
      }
      // Activate ACC
      if(btSignal == 'x'){
        if(ACC_STATE == 0){
          ACC_STATE = 1;
          serialBT.println("ACC ON");
        }
      }
      // Deactivate ACC
      if(btSignal == 'y'){
        if(ACC_STATE == 1){
          ACC_STATE = 0;
          serialBT.println("ACC OFF");
        }
      }
      // Activate TSDR
      if(btSignal == 'z'){
        if(TSDR_STATE == 0){
          TSDR_STATE = 1;
          serialBT.println("TSDR ON");
        }
      }
      // Deactivate TSDR
      if(btSignal == 'q'){
        if(TSDR_STATE == 1){
          TSDR_STATE = 0;
          serialBT.println("TSDR OFF");
        }
      }  
      // Activate LKA
      if(btSignal == 'a'){
        if(LKA_STATE == 0){
          LKA_STATE = 1;
          serialBT.println("LKA ON");
        }
      } 
      // Deactivate LKA
      if(btSignal == 'e'){
        if(LKA_STATE == 1){
          LKA_STATE = 0;
          serialBT.println("LKA OFF");
        }
      } 
      // Activate ALS
      if(btSignal == 'g'){
        if(ALS_STATE == 0){
          ALS_STATE = 1;
          serialBT.println("ALS ON");
        }
      } 
      // Deactivate ALS
      if(btSignal == 'w'){
        if(ALS_STATE == 1){
          ALS_STATE = 0;
          serialBT.println("ALS OFF");
          // Turn OFF Front LEDs
          analogWrite(LOW_BEAM_LED, 0);
          digitalWrite(HIGH_BEAM_LED, LOW);
        }
      } 
      // Turn OFF Front LEDs
      if(btSignal == 'c'){
        if(ALS_STATE == 1){
          ALS_STATE = 0;
          serialBT.println("ALS OFF");
        }
        analogWrite(LOW_BEAM_LED, 0);
        digitalWrite(HIGH_BEAM_LED, LOW);
      }
      // Turn ON Day Running Lights
      if(btSignal == 't'){
        if(ALS_STATE == 1){
          ALS_STATE = 0;
          serialBT.println("ALS OFF");
        }
        analogWrite(LOW_BEAM_LED, DAY_RUNNING_LED);
        digitalWrite(HIGH_BEAM_LED, LOW);
      }
      // Turn ON Low Beam LED
      if(btSignal == 'j'){
        if(ALS_STATE == 1){
          ALS_STATE = 0;
          serialBT.println("ALS OFF");
        }
        analogWrite(LOW_BEAM_LED, 255);
        digitalWrite(HIGH_BEAM_LED, LOW);
      }
      // Turn ON HIGH Beam LED
      if(btSignal == 'p'){
        if(ALS_STATE == 1){
          ALS_STATE = 0;
          serialBT.println("ALS OFF");
        }
        analogWrite(LOW_BEAM_LED, 255);
        digitalWrite(HIGH_BEAM_LED, HIGH);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Bluetooth Recall FreeRTOS Function
void BT_Control(void *pvParameters){
  while(1){
    if(Recall_Signal == 'f')  Forward();
    if(Recall_Signal == 'b')  Backward();
    if(Recall_Signal == 's')  Stop();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Blinkers FreeRTOS Function
void Blinker_LEDs(void *pvParameters){
  while(1){
    if(LEFT_BLINKER_STATE == 0){ 
      digitalWrite(LEFT_LED_BLINKER, LOW);
    }
    if(RIGHT_BLINKER_STATE == 0){ 
      digitalWrite(RIGHT_LED_BLINKER, LOW);
    }
    if(LEFT_BLINKER_STATE == 1 && RIGHT_BLINKER_STATE == 0){
      digitalWrite(LEFT_LED_BLINKER, BLINKER_VALUE);
      BLINKER_VALUE =! BLINKER_VALUE;
    }
    if(LEFT_BLINKER_STATE == 0 && RIGHT_BLINKER_STATE == 1){
      digitalWrite(RIGHT_LED_BLINKER, BLINKER_VALUE);
      BLINKER_VALUE =! BLINKER_VALUE;
    }
    if(LEFT_BLINKER_STATE == 1 && RIGHT_BLINKER_STATE == 1){
      digitalWrite(LEFT_LED_BLINKER, BLINKER_VALUE);
      digitalWrite(RIGHT_LED_BLINKER, BLINKER_VALUE);
      BLINKER_VALUE =! BLINKER_VALUE;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// AEB and ACC FreeRTOS Function
void AEB_ACC(void *pvParameters){
  while(1){
    digitalWrite(FRONT_ULTRASONIC_TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_ULTRASONIC_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_ULTRASONIC_TRIGGER_PIN, LOW);
    unsigned long pulseDuration = pulseIn(FRONT_ULTRASONIC_ECHO_PIN, HIGH);
    float Front_Distance_CM = pulseDuration * 0.0343 / 2;

    // Autonomous Emergency Braking (AEB)
    if(Front_Distance_CM > 2 && Front_Distance_CM <= AEB_Threshold && Recall_Signal == 'f'){
      if(AEB_Counter == 0){
        Stop();
        Recall_Signal = 's';
        LEFT_BLINKER_STATE = 1;
        RIGHT_BLINKER_STATE = 1;
        AEB_Counter = 1;
        serialBT.println("STOP (AEB)");
        // Force STOP ACC
        if(ACC_STATE == 1){
          ACC_STATE = 0;
          serialBT.println("ACC OFF");
        }
        // Force STOP LKA
        if(LKA_STATE == 1){
          LKA_STATE = 0;
          serialBT.println("LKA OFF");
        }
      }
    }

    // Adaptive Cruise Control (ACC)
    if(ACC_STATE == 1){
      // Decrease Speed
      if(Front_Distance_CM > AEB_Threshold && Front_Distance_CM <= ACC_Decrease_Threshold){
        if(SPEED > 130){
          OLD_SPEED = SPEED;
          SPEED = OLD_SPEED - 5;
          if(OLD_SPEED != SPEED){
            serialBT.print("Speed = ");
            serialBT.println(SPEED);
          }
        }
      }
      // Increase Speed
      if(Front_Distance_CM > ACC_Increase_Threshold){
        if(SPEED < MAX_SPEED){
          OLD_SPEED = SPEED;
          SPEED = OLD_SPEED + 5;
          if(OLD_SPEED != SPEED){
            serialBT.print("Speed = ");
            serialBT.println(SPEED);
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

// TSDR Functions from RaspberryPi I2C communication protocol
// I2C receive event
void receiveEvent(int howMany) {
  while (Wire.available()) {
    int signal = Wire.read();
    xQueueSend(signalQueue, &signal, portMAX_DELAY);
  }
}
// FreeRTOS task to handle I2C signals
void i2cTask(void *pvParameters) {
  int receivedSignal;
  while (1) {
    if (xQueueReceive(signalQueue, &receivedSignal, portMAX_DELAY) == pdPASS) {
      handleSignal(receivedSignal);
    }
  }
}
// TSDR Function
void handleSignal(int signal) {
  if(TSDR_STATE == 1){
    switch(signal){
      // Speed Limit 60 km/h Sign
      case 1:
        MAX_SPEED = SPEED_LIMIT_60;
        serialBT.println("Speed Limit 60 km/h");
        serialBT.println("Speed Limit 180 PWM");
        if(SPEED > 180){
          SPEED = 180;
        }
        break;
      // Speed Limit 120 km/h Sign
      case 2:
        MAX_SPEED = SPEED_LIMIT_120;
        serialBT.println("Speed Limit 120 km/h");
        serialBT.println("Speed Limit 220 PWM");
        if(SPEED > 220){
          SPEED = 220;
        }
        break;
      // STOP Sign
      case 3:
        Last_BT_STOP = Recall_Signal;
        Stop();
        Recall_Signal = 's';
        serialBT.println("STOP Sign");
        delay(STOP_Time);
        MAX_SPEED = NO_SPEED_LIMIT;
        serialBT.println("No Speed Limit");
        serialBT.println("Speed Limit 255 PWM");
        Recall_Signal = Last_BT_STOP;
        serialBT.println("TSDR OFF");
        TSDR_STATE = 0;
        break;
    }
  }
}

// LKA FreeRTOS Function
void LKA(void *pvParameters){
  while(1){
    if(LKA_STATE == 1){
      int RIGHT_IR_READING = digitalRead(RIGHT_IR);
      int LEFT_IR_READING = digitalRead(LEFT_IR);
      // IN LANE
      if(RIGHT_IR_READING == HIGH && LEFT_IR_READING == HIGH){
        SteeringServo.write(STRAIGHT);
        LKA_Counter = 0;
      }
      // Turn Left
      else if(RIGHT_IR_READING == LOW && LEFT_IR_READING == HIGH){
        SteeringServo.write(LKA_LEFT);
        LKA_Counter = 0;
      }
      // Turn Right
      else if(RIGHT_IR_READING == HIGH && LEFT_IR_READING == LOW){
        SteeringServo.write(LKA_RIGHT);
        LKA_Counter = 0;
      }
      // ERROR Readings
      if(RIGHT_IR_READING == LOW && LEFT_IR_READING == LOW){
        SteeringServo.write(STRAIGHT);
        LKA_Counter ++;
        serialBT.println("LKA ERROR");
        if(LKA_Counter >= 3){
          LKA_STATE = 0;
          LKA_Counter = 0;
          serialBT.println("LKA OFF");
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Right BSW FreeRTOS Function
void Right_BSW(void *pvParameters){
  while(1){
    digitalWrite(ULTRASONIC_TRIGGER_RIGHT_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIGGER_RIGHT_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIGGER_RIGHT_PIN, LOW);
    unsigned long pulseDuration = pulseIn(ULTRASONIC_ECHO_RIGHT_PIN, HIGH);
    float Right_Distance = pulseDuration * 0.0343 / 2;
    if(Right_Distance <= BSW_Distance){
      digitalWrite(RIGHT_BSW_LED, RBSW_LED_VALUE);
      RBSW_LED_VALUE =! RBSW_LED_VALUE;
    }
    else{
      digitalWrite(RIGHT_BSW_LED, LOW);
      RBSW_LED_VALUE = 1;
    }
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

// Left BSW FreeRTOS Function
void Left_BSW(void *pvParameters){
  while(1){
    digitalWrite(ULTRASONIC_TRIGGER_LEFT_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIGGER_LEFT_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIGGER_LEFT_PIN, LOW);
    unsigned long pulseDuration = pulseIn(ULTRASONIC_ECHO_LEFT_PIN, HIGH);
    float Left_Distance = pulseDuration * 0.0343 / 2;
    if(Left_Distance <= BSW_Distance){
      digitalWrite(LEFT_BSW_LED, LBSW_LED_VALUE);
      LBSW_LED_VALUE =! LBSW_LED_VALUE;
    }
    else{
      digitalWrite(LEFT_BSW_LED, LOW);
      LBSW_LED_VALUE = 1;
    }
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

// ALS FreeRTOS Function
void ALS(void *pvParameters){
  while(1){
    if(ALS_STATE == 1){
      int LDR_Value = analogRead(ANALOG_LDR_PIN);
      // Very bright ambient environment -> Turn OFF all LEDs
      if(LDR_Value < ALS_Threshold_1){
        analogWrite(LOW_BEAM_LED, 0);
        digitalWrite(HIGH_BEAM_LED, LOW);
      }
      // Bright ambient environment -> Turn ON Day Running Lights
      if(LDR_Value > ALS_Threshold_1 && LDR_Value < ALS_Threshold_2){
        analogWrite(LOW_BEAM_LED, DAY_RUNNING_LED);
        digitalWrite(HIGH_BEAM_LED, LOW);
      }
      // Dark ambient environment -> Turn ON Low Beam LED
      if(LDR_Value > ALS_Threshold_2 && LDR_Value < ALS_Threshold_3){
        analogWrite(LOW_BEAM_LED, 255);
        digitalWrite(HIGH_BEAM_LED, LOW);
      }
      // Very dark ambient environment -> Turn ON High Beam LED
      if(LDR_Value > ALS_Threshold_3){
        analogWrite(LOW_BEAM_LED, 255);
        digitalWrite(HIGH_BEAM_LED, HIGH);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));  
  }
}

void setup(){
  Serial.begin(115200);
  // Bluetooth device Setup
  serialBT.begin("FARAG - ADAS");
  // Create I2C queue
  signalQueue = xQueueCreate(10, sizeof(int));
  if (signalQueue == NULL) {
    Serial.println("Failed to create queue");
    while (1); // Stop if queue creation failed
  }
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent); 
  // L298N Pins Setup
  pinMode(EN_A, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);
  pinMode(EN_B, OUTPUT);
  // SG90 Servo Pins Setup
  SteeringServo.attach(SteeringPin);
  SteeringServo.write(STRAIGHT);
  // Front Ultrasonic Sensor Pins Setup
  pinMode(FRONT_ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(FRONT_ULTRASONIC_ECHO_PIN, INPUT);
  // Blinkers LEDs Pins Setup
  pinMode(BACK_STOP_LED, OUTPUT);
  pinMode(LEFT_LED_BLINKER, OUTPUT);
  pinMode(RIGHT_LED_BLINKER, OUTPUT);
  // IR Sensors Pins Setup
  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);
  // Left Ultrasonic Sensor and LED Pins Setup
  pinMode(ULTRASONIC_TRIGGER_LEFT_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_LEFT_PIN, INPUT);
  pinMode(LEFT_BSW_LED, OUTPUT);
  // Right Ultrasonic Sensor and LED Pins Setup
  pinMode(ULTRASONIC_TRIGGER_RIGHT_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_RIGHT_PIN, INPUT);
  pinMode(RIGHT_BSW_LED, OUTPUT);
  // LDR and Front LEDs Pins Setup
  pinMode(ANALOG_LDR_PIN, INPUT);
  pinMode(HIGH_BEAM_LED, OUTPUT);
  pinMode(LOW_BEAM_LED, OUTPUT);
  // FreeRTOS Task Creation
  xTaskCreate(BT_Receive,"BT_Receive",3000,NULL,2,NULL);
  xTaskCreate(BT_Control,"BT_Control",3000,NULL,2,NULL);
  xTaskCreate(Blinker_LEDs,"Blinker_LEDs",3000,NULL,1,NULL);
  xTaskCreate(AEB_ACC,"AEB_ACC",4000,NULL,4,NULL);
  xTaskCreate(i2cTask,"I2C Task",3000,NULL,3,NULL);
  xTaskCreate(LKA,"LKA",3000,NULL,3,NULL);
  xTaskCreate(Right_BSW,"Right_BSW",3000,NULL,3,NULL);
  xTaskCreate(Left_BSW,"Left_BSW",3000,NULL,3,NULL);
  xTaskCreate(ALS,"ALS",3000,NULL,1,NULL);
}

void loop(){

}