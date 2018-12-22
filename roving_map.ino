
#include <TinyGPS++.h>
boolean validGPS = false;
#define ConsoleBaud 115200
TinyGPSPlus gps;
#define rounder 20 //Number of steps per revolution in this motor
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
int oldMoveLng=0;
int oldMoveLat=0;
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *myMotor2 = AFMS.getStepper(20, 2);
Adafruit_StepperMotor *myMotor1 = AFMS.getStepper(20, 1);
/* The following are the metal-points under the photo.  After settling the sat photo on the two pins
 *  look them up carefully on either google earth or latlng.net and check their respective lat/lng and 
 *  put them into the appropiate spot.  Use four positions after decimal for accuracy.
 */
float leftUpperLat=61.1732;
float leftUpperLng=-149.9043;
float rightLowerLat=61.09133;
float rightLowerLng=-149.7339;
//This sets the home position from which all subsequent measurements are made
float homeLat=rightLowerLat;
float homeLng=rightLowerLng;
int rangeLng=abs(float(((leftUpperLng-rightLowerLng)*10000)));//calculates lng distance on map
int rangeLat=float(((leftUpperLat-rightLowerLat)*10000));//calculates lat distance on map
unsigned long lastUpdateTime = 0;

void setup()
{
  Serial1.begin(9600); 
  AFMS.begin(); 
  myMotor1->setSpeed(10);  // 10 rpm
  myMotor2->setSpeed(10);  // 10 rpm 
  //This sets the limit switches at the ends of the stepper runs
  pinMode(12,INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP); 
  pinMode(13,HIGH);
  pinMode(9, OUTPUT); //Sets up the two tiny leds on the board 
   for(int i=0;i<150;i++)
   {
    analogWrite(9,i);
    delay(30);
   }
   digitalWrite(9,HIGH);
 //This sets the intitial position of the pointer arms
 while(digitalRead(12))
 {
  myMotor2->step(rounder,BACKWARD,DOUBLE);     
 }
 myMotor2->step(rounder*4,FORWARD,DOUBLE);
 while(digitalRead(13))
 {
  myMotor1->step(rounder,BACKWARD,DOUBLE);
 }
 myMotor1->step(rounder*4,FORWARD,DOUBLE);
 myMotor2->step(rounder*128,FORWARD,DOUBLE);
}
 
void loop()
{
   if (Serial1.available())
  {
    validGPS = gps.encode(Serial1.read());
  }
 if (validGPS)
  {
   // Every 3 seconds, do an update on GPS position--this can obviously be changed depending on how fast your going
   if (millis() - lastUpdateTime >= 3000)
   {
    lastUpdateTime = millis();
    // Troubleshooting
    Serial.print("gps lat");
    Serial.println(gps.location.lat());
    Serial.print("gps lng");
    Serial.println(gps.location.lng());
    //Makes sure you dont do anything if you fall off the map
    if((gps.location.lat()<leftUpperLat)&&(gps.location.lat()>rightLowerLat)&&(gps.location.lng()<rightLowerLng)
    &&(gps.location.lng()>leftUpperLng))
    {
      float moveLat=(gps.location.lat()-homeLat)*10000;
      float moveLng=abs((gps.location.lng()-homeLng)*10000);
      Serial.print("moveLat");
      Serial.println(moveLat);
      Serial.print("moveLng");
      Serial.println(moveLng);
      //The limit numbers in these are determined by counting the number of turns until you reach the arms limits
      int newMoveLat=map(moveLat,0,rangeLat,0,128);
      newMoveLat=constrain(newMoveLat,0,120);
      int newMoveLng=map(moveLng,0,rangeLng,0,123);
      newMoveLng=constrain(newMoveLng,0,120);
      int stepMoveLat=newMoveLat-oldMoveLat;
      int stepMoveLng=newMoveLng-oldMoveLng;
      Serial.print("stepMoveLat");
      Serial.println(stepMoveLat);
      Serial.print("stepMoveLng");
      Serial.println(stepMoveLng);
      if(stepMoveLat!=0)
      {
        if(stepMoveLat<0)
        {
          myMotor2->step((rounder*abs(stepMoveLat)),FORWARD, DOUBLE);
        }
        else
        {
          myMotor2->step((rounder*stepMoveLat),BACKWARD,DOUBLE);
        }
       }
       
      if(stepMoveLng!=0)
      {
        if(stepMoveLng<0)
        {
          myMotor1->step((rounder*abs(stepMoveLng)),BACKWARD,DOUBLE);
        }
        else
        {
          myMotor1->step((rounder*stepMoveLng),FORWARD,DOUBLE);
        }
      }
      
      oldMoveLng=newMoveLng;
      oldMoveLat=newMoveLat;
      //Saves energy....
      myMotor1->release();
      myMotor2->release();      
    }
   }
   
  }  
}

