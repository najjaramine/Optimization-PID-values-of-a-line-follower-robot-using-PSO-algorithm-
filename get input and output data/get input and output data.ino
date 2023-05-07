#include <QTRSensors.h>
#include <SD.h>
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
QTRSensors qtr;
File file;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
//PID control system variables
float Kp = 0.07;
float Ki = 0.0008; 
float Kd = 0.6; 
int P;
int I;
int D;
int lastError = 0;
unsigned long previousMillis = 0;
unsigned long interval = 10.00;
unsigned long currentMillis;
int CS_PIN = 53;
// initialization of maxspeed and basedspeed
const uint8_t max_speed = 90;
const uint8_t base_speed = 70;
//motors declarations
#define ENA 13
#define IN1 12
#define IN2 11
#define IN3 10
#define IN4 9
#define ENB 8
#define calibration_button 2
const int buzzer = 7; //buzzer to arduino pin 7
bool test=false;
int errorTab[500];
int motorSpeedTab[500];
int j=0;
int error;
int motorspeed;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Starts the serial communication
  //calibrate button and led declarations
  pinMode(calibration_button, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT); // Set buzzer - pin 7 as an output
   pinMode(13, OUTPUT);
  //motors declarations
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  //qtr 8rc declarations  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
  A0, A1, A2, A3, A4, A5, A6, A7
  }, SensorCount);

    calibrate();
    delay(2000);
    
 
  while (1) {
    if (millis()>16500) {
       stop();
       delay(3000);
       digitalWrite(13, HIGH);
       delay(2000);
       digitalWrite(13, LOW);
       storeData();
       tone(buzzer, 3000); 
       delay(2000);
       noTone(buzzer); 
       break;
    }
    else
     PID_control() ;
  }
 
  
}

void loop() {
  // put your main code here, to run repeatedly
}

void calibrate(){
  delay(1500);
  for (int i = 0; i < 150; i++) {
    qtr.calibrate();
    delay(20);
    }   
  tone(buzzer, 3000); 
  delay(1000);
  noTone(buzzer);       
}

void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues); //read the current position
  int error = 3500 - position; //3500 is the ideal position (the centre)
  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd; //calculate the correction
                                       //needed to be applied to the speed
  
  int motor_speeda = base_speed + motorspeed;
  int motor_speedb =  base_speed - motorspeed;
  
  if (motor_speeda > max_speed) {
    motor_speeda = max_speed;
  }
  if (motor_speedb > max_speed) {
    motor_speedb = max_speed;
  }
  if (motor_speeda < 0) {
    motor_speeda = 0;
  }
  if (motor_speedb < 0) {
    motor_speedb = 0;
  } 
  forward_brake(motor_speeda, motor_speedb);

  currentMillis = millis();
  if ((currentMillis - previousMillis) >= interval) {
    previousMillis = currentMillis;
    errorTab[j]=error;
    motorSpeedTab[j]=motorspeed;
    j++;
  }
}

void forward_brake(int motor_speeda,int motor_speedb){
  analogWrite(ENA,motor_speeda);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  analogWrite(ENB,motor_speedb);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void stop(){
  analogWrite(ENA,0);
  analogWrite(ENB,0);
}

void initializeSD()
{
  Serial.println("Initializing SD card...");
  pinMode(CS_PIN, OUTPUT);

  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }
}

int createFile(char filename[])
{
  file = SD.open(filename, FILE_WRITE);

  if (file)
  {
    Serial.println("File created successfully.");
    return 1;
  } else
  {
    Serial.println("Error while creating file.");
    return 0;
  }
}

int writeToFile(char text[])
{
  if (file)
  {
    file.println(text);
    Serial.println("Writing to file: ");
    Serial.println(text);
    return 1;
  } else
  {
    Serial.println("Couldn't write to file");
    return 0;
  }
}

void closeFile()
{
  if (file)
  {
    file.close();
    Serial.println("File closed");
  }
}

void storeData()
{
  char StringError[10];
  char StringMotorSpeed[15];
  initializeSD();
  for(int k=0;k<=1;k++ ){
     sprintf(StringError, "%d", errorTab[k]);  
     createFile("errors.txt");
     writeToFile(StringError);
     closeFile();
  }

  for(int m=0;m<=10;m++ ){
     sprintf(StringMotorSpeed, "%d", motorSpeedTab[m]);
     createFile("motorSpeed.txt");
     writeToFile(StringMotorSpeed);
     closeFile();
  }
}
