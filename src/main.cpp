/*Lorsque une présence est détecté, on allume une LED IR 1sec que 
la balise du chat recoit, cette dernière nous envoie un code 
NEC-IR et on le lit grâce au photorécepteur
Si tout est beau (code NEC de 0x1234), on ouvre la porte (servo)*/

#include <Arduino.h>
#include <Wire.h>
#include "Vl53lxWrapper.h"
#include <IRremote.h> 
#include <ESP32Servo.h>
#include <SPI.h>
#include <U8g2lib.h>

//PINS
#define PIN_RECV 2 //photorécepteur
#define PIN_ACTI 1 //del IR
int xshutPin = 7;
int interruptPin = 6;
int sdaPin = 8;
int sclPin = 9;

//servo
Servo myservo;
int pin = 17;
bool openDoor = false;
bool doorOpened = false;
int counterDoorOpen = 0;

//Distance
Vl53lxWrapper *vl53lxWrapper;
int sensorDeviceAddress = 0x12;
int distance;
int distsanceMin = 300; //distance min pour détecter une presence
bool measureDistance = false;

//Communication
bool commToStart = false;
int counterLEDUptime = 0;

//écran
U8G2_SSD1322_NHD_256X64_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/36, /* data=*/35, /* cs=*/34, /* dc=*/20, /* reset=*/19); // Enable U8G2_16BIT in u8g2.h


// Timer
hw_timer_t *timer = NULL;
int timerCount = 0; // timer pour le save dans la SD
volatile bool interruptbool1 = false;


void IRAM_ATTR onTimer()  //cmompteur de 500ms
{
  interruptbool1 = true; // Indicates that the interrupt has been entered since the last time its value was changed to false
  counterLEDUptime++;
  counterDoorOpen++;
  measureDistance = true;
}

void setup()
{
  Serial.begin(115200);

  // timer
  timer = timerBegin(0, 80, true);             // Begin timer with 1 MHz frequency (80MHz/80)
  timerAttachInterrupt(timer, &onTimer, true); // Attach the interrupt to Timer1
  unsigned int timerFactor = 500000;           //500ms
  timerAlarmWrite(timer, timerFactor, true);   // Initialize the timer
  timerAlarmEnable(timer);

    // servo
   ESP32PWM::allocateTimer(1);
  myservo.setPeriodHertz(50);      // standard 50 hz servo
  myservo.attach(pin, 1000, 2000); // attaches the servo on pin 18 to the servo object
  myservo.write(0); //ferme porte

  //IR et distance
  vl53lxWrapper = new Vl53lxWrapper(xshutPin,
                                    interruptPin,
                                    sensorDeviceAddress,
                                    sdaPin,
                                    sclPin);
  IrReceiver.begin(PIN_RECV); // Initializes the IR receiver object
  pinMode(PIN_ACTI, OUTPUT);

  //ecran
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();                    // clear the internal memory
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.drawStr(10, 20, "Door close");         // write something to the internal memory
  u8g2.sendBuffer();
}

void loop()
{

  /*if (IrReceiver.decode()) { //decode IR recu de la balise
    Serial.println("Received something...");    
    IrReceiver.printIRResultShort(&Serial); // Prints a summary of the received data
    Serial.println();
    if(IrReceiver.decodedIRData.protocol == NEC && IrReceiver.decodedIRData.address==0x12 && IrReceiver.decodedIRData.command==0x34){ //check le protocole et adresse et commande envoyer
      openDoor = true;
    }
    IrReceiver.resume(); // Important, enables to receive the next IR signal
  }  */

  /*if(openDoor){
    myservo.write(180);
    counterDoorOpen = 0;
    doorOpened = true;
    u8g2.clearBuffer();                    // clear the internal memory
    u8g2.drawStr(10, 20, "Door open");         // write something to the internal memory
    u8g2.sendBuffer();
    openDoor = false;
  }

  if(doorOpened){
    Serial.println("counter dooropen : " + (String)counterDoorOpen);
  }
  if(doorOpened && counterDoorOpen>20 && distance >= distsanceMin){ //ferme la porte après 10 seconde si il ny a plus personne
      myservo.write(0);
      doorOpened = false;
      u8g2.clearBuffer();                    // clear the internal memory
      u8g2.drawStr(10, 20, "Door close");         // write something to the internal memory
      u8g2.sendBuffer();
  }
  else if(doorOpened && counterDoorOpen>10 && distance <= distsanceMin){ //quelqun mange toujours
    counterDoorOpen = 0;
  }*/

  if(measureDistance){ //mesure distance au 500ms
    VL53LX_MultiRangingData_t measurement = vl53lxWrapper->getLatestMeasurement();
    distance = measurement.RangeData->RangeMilliMeter;
    Serial.println(distance);
    measureDistance = false;
  }
/*
  if(distance <= distsanceMin){ //Présence détecté : allume DEL IR
    commToStart = true;
    counterLEDUptime = 0;
    digitalWrite(PIN_ACTI, HIGH);
  }
  if(counterLEDUptime>2 && commToStart == true){ //Fini de flasher la LED IR pour 1 sec
    digitalWrite(PIN_ACTI, LOW);
    commToStart = false;
    counterLEDUptime = 0;
  }*/
}
