#include <Arduino.h>
#include <Wire.h>
#include "Vl53lxWrapper.h"
#include <IRremote.h> 
#include <ESP32Servo.h>
#include <SPI.h>
#include <U8g2lib.h>

//PINS
#define PIN_IR_RX 13      //photorécepteur
#define PIN_IR_TX 17      //del IR
#define PIN_INT_DIST 8    //interrupt capteur de distance
#define PIN_SDA 8         //SDA
#define PIN_SCL 9         //SCL
#define PIN_SERVO 5       //servomoteur porte

//servo
Servo myservo;
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
U8G2_SSD1322_NHD_256X64_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/36, /* data=*/35, /* cs=*/19, /* dc=*/20, /* reset=*/34); // Enable U8G2_16BIT in u8g2.h


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
  myservo.attach(PIN_SERVO, 1000, 2000); // attaches the servo on pin 18 to the servo object
  myservo.write(0); //ferme porte

  //IR et distance
  vl53lxWrapper = new Vl53lxWrapper(45, //pas connecté
                                    PIN_INT_DIST,
                                    sensorDeviceAddress,
                                    PIN_SDA,
                                    PIN_SCL);
  IrReceiver.begin(PIN_IR_RX); // Initializes the IR receiver object
  pinMode(PIN_IR_TX, OUTPUT);

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
