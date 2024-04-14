#include <Arduino.h>
#include <Wire.h>
#include <IRremote.h> 
#include <ESP32Servo.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <pins.h>
#include <svg.h>
#include <WiFi.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <VL53L0X.h>
#include "time.h"

//flags systeme
bool criticalError = false;

//balance 
// conversion speeds ((Continuous) Samples Per Second)
//single:
#define MAX_CMD_CONV    0x80        // 0b10000000
#define MAX_SPS_1       0x00
#define MAX_SPS_2_5     0x01
#define MAX_SPS_5       0x02
#define MAX_SPS_10      0x03
#define MAX_SPS_15      0x04
#define MAX_SPS_30      0x05
#define MAX_SPS_60      0x06
#define MAX_SPS_120     0x07
// continuous:
#define MAX_CSPS_60     0x04
#define MAX_CSPS_120    0x05
#define MAX_CSPS_240    0x06
#define MAX_CSPS_480    0x07

// registers: 
#define MAX_CMD_REG     0xC0        // 0b11000000
// register addresses:
#define MAX_STAT1       0x00 << 1
#define MAX_CTRL1       0x01 << 1
#define MAX_CTRL2       0x02 << 1
#define MAX_CTRL3       0x03 << 1
#define MAX_DATA        0x04 << 1
#define MAX_SOC         0x05 << 1
#define MAX_SGC         0x06 << 1
#define MAX_SCOC        0x07 << 1
#define MAX_SCGC        0x08 << 1
#define MAX_READ        0x01
#define MAX_WRITE       0x00

#define MAX_UNIPOLAR    0x40        // input range: defaults to bipolar (-AREF to +AREF). UNI = (0 to +AREF)
#define MAX_CONTCONV    0x02        // defaults to single-conversion. FYI the first 3 data from continuous are incorrect. 

//WiFi
bool wifiConnected = false;
const char* ssid     = "Test";
const char* password = "test1234";

//Time
String hourNow = "00:00";
int RTCTimeHour = 0;
int RTCTimeMinutes =  0;
int RTCTimeSeconds = 0; 
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   Offset_sec = -14400; //-4h
tmElements_t tm;

//capteur de distance
VL53L0X sensor;
const int distanceMin = 325;

//servo-porte
Servo myservo;
bool doorOpened = false; //flag si porte ouverte
bool doorOpenedByCat = false; //flag si porte ouverte par chat
bool doorOpenedByManually = false; //flag si porte ouverte manuellement
int counterDoorOpenCatLeft = 0; //compte le nb de seconde depuis que le chat est parti

//Communication IR
enum StatesComm  {STANDBY, TX_COMM, RX_COMM, CAT_EATING};
StatesComm stateComm = STANDBY;
int counterTXUptime = 0;
int counterRXUptime = 0;

//Portions
int lastFeeding = 1;
int feedingPortions [2] = {0,0};
String timeFeeding1 = "08:00";
String timeFeeding2 = "00:00";

//Batteries
uint8_t batTag = 0; //batterie balise 0 à 255
uint8_t batDis = 0; //batterie distributeur 0 à 255

//Interface utilisateur
U8G2_SSD1322_NHD_256X64_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/PIN_SCK, /* data=*/PIN_MOSI, /* cs=*/PIN_CS_SCREEN, /* dc=*/PIN_DC_SCREEN, /* reset=*/PIN_RESET_SCREEN); // Enable U8G2_16BIT in u8g2.h
enum Menus  {NAVIGATION_MENU, MAIN_MENU, PORTIONS_MENU, HOUR_MENU, ACTIONS_MENU};
Menus menu_selected = MAIN_MENU;
int Item_selected_row = 1;
int Item_selected_column = 1;
int feedingSelected = 1;
bool keypad_up = false;
bool keypad_down = false;
bool keypad_left = false;
bool keypad_right = false;
bool keypad_select = false;
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 200;    // the debounce time; increase if the output flickers
int lastMinutes; //dernière minutes pour savoir si on rafraichi l'heure

// Timer
hw_timer_t *timer = NULL;
volatile bool interruptbool1 = false;
int timerSecond =  0; //pair si timer tombe sur 1 seconde


void IRAM_ATTR onTimer()  //compteur de 500ms
{
  interruptbool1 = true; // Indicates that the interrupt has been entered since the last time its value was changed to false

  if(timerSecond % 2 == 0){ //1 seconde
      if(stateComm == RX_COMM){
        counterRXUptime++;
      }
      if(doorOpenedByCat){
        counterDoorOpenCatLeft++;
      }
  }
  /*if(timerSecond % 6 == 0){ //3 seconde
    commToStart = true;
  }*/
  if(stateComm == TX_COMM){
    counterTXUptime++;
  }
  timerSecond++;
}

void keypadUPInterrupt(){
  if((millis() - lastDebounceTime) > debounceDelay && (menu_selected == HOUR_MENU || menu_selected == ACTIONS_MENU || menu_selected == PORTIONS_MENU)){
    keypad_up = true;
    lastDebounceTime = millis();
  }
}

void keypadDOWNInterrupt(){
  if((millis() - lastDebounceTime) > debounceDelay && (menu_selected == HOUR_MENU || menu_selected == ACTIONS_MENU || menu_selected == PORTIONS_MENU)){
    keypad_down = true;
    lastDebounceTime = millis();
  }
}

void keypadLEFTInterrupt(){
  if((millis() - lastDebounceTime) > debounceDelay && (menu_selected == HOUR_MENU || menu_selected == NAVIGATION_MENU || menu_selected == ACTIONS_MENU || menu_selected == PORTIONS_MENU)){
    keypad_left = true;
    lastDebounceTime = millis();
  }
}

void keypadRIGHTInterrupt(){
  if((millis() - lastDebounceTime) > debounceDelay && (menu_selected == HOUR_MENU || menu_selected == NAVIGATION_MENU || menu_selected == ACTIONS_MENU || menu_selected == PORTIONS_MENU)){
    keypad_right = true;
    lastDebounceTime = millis();
  }
}

void keypadSELECTInterrupt(){
  if((millis() - lastDebounceTime) > debounceDelay){
    keypad_select = true;
    lastDebounceTime = millis();
  }
}


String return2digits(int number) {
  String ret = "";
  if (number >= 0 && number < 10) {
    ret += '0';
  }
  ret += String(number);
  return ret;
}

void printMenu(Menus ms){

  u8g2.clearBuffer();

  switch(ms){
    case NAVIGATION_MENU :
      u8g2.drawXBMP(0,0,50,50,home_icon);
      u8g2.drawFrame(2,2,46,46);
      u8g2.setFont(u8g_font_helvB10);
      u8g2.drawStr(0, 60, "Accueil"); 

      u8g2.drawXBMP(68,0,50,50,food_icon);
      //u8g2.drawFrame(70,2,46,46);
      u8g2.drawStr(65, 60, "Portions");

      u8g2.drawXBMP(136,0,50,50,hour_icon);
      //u8g2.drawFrame(138,2,46,46);
      u8g2.drawStr(140, 60, "Heure");

      u8g2.drawXBMP(204,0,50,50,action_icon);
      //u8g2.drawFrame(206,2,46,46);
      u8g2.drawStr(204, 60, "Actions");
      break;

    case MAIN_MENU : 
      lastMinutes = RTCTimeMinutes;
      u8g2.setFont(u8g2_font_logisoso30_tf);
      u8g2.drawStr(0, 40, hourNow.c_str());   
      u8g2.setFont(u8g2_font_logisoso16_tf);
      u8g2.drawStr(140, 20, ("Repas:   " + (String)lastFeeding).c_str());        
      u8g2.drawStr(140, 40, ("Portions: " + (String)feedingPortions[lastFeeding-1]).c_str());     
      if(wifiConnected){ 
        u8g2.drawXBMP(0,50,15,15,wifi_icon);
      }
      else{
        u8g2.drawXBMP(0,50,15,15,no_wifi_icon);
      }
      //batterie distributeur
      if(batDis == 0){ //0
        u8g2.drawXBMP(45,45,25,25,battery_empty_icon);
      }
      else if(batDis > 0 && batDis <= 30){ //15
        u8g2.drawXBMP(45,45,25,25,battery_low_icon);
      }
      else if(batDis > 31 && batDis <= 95){ //63
        u8g2.drawXBMP(45,45,25,25,battery_one_quarter_icon);
      }
      else if(batDis > 95 && batDis <= 159){ //127
        u8g2.drawXBMP(45,45,25,25,battery_half_icon);
      }
      else if(batDis > 159 && batDis <= 223){ //191
        u8g2.drawXBMP(45,45,25,25,battery_three_quarter_icon);
      }
      else if(batDis > 223 && batDis <= 255){ //255
        u8g2.drawXBMP(45,45,25,25,battery_full_icon);
      }
      //batterie balise
      if(batTag == 0){ //0
        u8g2.drawXBMP(90,45,25,25,battery_empty_icon);
      }
      else if(batTag > 0 && batTag <= 30){ //15
        u8g2.drawXBMP(90,45,25,25,battery_low_icon);
      }
      else if(batTag > 31 && batTag <= 95){ //63
        u8g2.drawXBMP(90,45,25,25,battery_one_quarter_icon);
      }
      else if(batTag > 95 && batTag <= 159){ //127
        u8g2.drawXBMP(90,45,25,25,battery_half_icon);
      }
      else if(batTag > 159 && batTag <= 223){ //191
        u8g2.drawXBMP(90,45,25,25,battery_three_quarter_icon);
      }
      else if(batTag > 223 && batTag <= 255){ //255
        u8g2.drawXBMP(90,45,25,25,battery_full_icon);
      }
      u8g2.setFont(u8g2_font_luBS12_tf);
      u8g2.drawStr(30, 63, "D");
      u8g2.drawStr(77, 63, "B");
      u8g2.drawXBMP(237,47,17,17,return_icon);
      u8g2.drawFrame(236,46,20,18);
      break;

    case PORTIONS_MENU:
      u8g2.setFont(u8g2_font_helvR14_tr);
      u8g2.drawStr(0, 17, "Portion:");
      u8g2.drawButtonUTF8(70,17, U8G2_BTN_INV, 0, 1, 1, "1");   
      u8g2.drawButtonUTF8(90,17, U8G2_BTN_BW1, 0, 0, 0, "2");
      u8g2.drawStr(0, 50, "Heure:");
      u8g2.drawStr(60,50, timeFeeding1.c_str());
      u8g2.drawXBMP(237,47,17,17,return_icon);
      u8g2.drawFrame(236,46,20,18); 
      break;

    case ACTIONS_MENU :   
      u8g2.setFont(u8g2_font_helvR14_tr);
      u8g2.drawStr(0, 17, "Porte:");
      u8g2.drawButtonUTF8(60,17, U8G2_BTN_INV, 0, 1, 1, "Ouvrir");   
      u8g2.drawButtonUTF8(120,17, U8G2_BTN_BW1, 0, 0, 0, "Fermer");
      u8g2.drawStr(0, 38, "Distribution:");
      u8g2.drawButtonUTF8(108,38, U8G2_BTN_BW1, 0, 0, 0, "Delivrer");
      u8g2.drawStr(0, 59, "Balise:"); 
      u8g2.drawButtonUTF8(65,59, U8G2_BTN_BW1, 0, 0, 0, "Ajouter");
      u8g2.drawButtonUTF8(134,59, U8G2_BTN_BW1, 0, 0, 0, "Reset"); 
      u8g2.drawXBMP(237,47,17,17,return_icon);
      u8g2.drawFrame(236,46,20,18);
      break;
    default : 
      break;
  }

  u8g2.sendBuffer(); 
}

void printNavigationMenuBorders(int selection_last, int selection_now){
  //enlever ancienne bordure de sélection
  if(selection_last >= 1 && selection_last<=4){
    u8g2.setDrawColor(0);
    int x1 = 2+(68*(selection_last-1));
    u8g2.drawFrame(x1,2,46,46);
    u8g2.updateDisplayArea(x1/8,0,7,6); //tuiles...
  }

  //ajouter nouvelle bordure de sélection
  if(selection_now >= 1 && selection_now<=4){
    u8g2.setDrawColor(1);
    int x2 = 2+(68*(selection_now-1));
    u8g2.drawFrame(x2,2,46,46);
    u8g2.updateDisplayArea(x2/8,0,7,6); //tuiles...
  }

}

void printActionsMenuButtons(){
  u8g2.clearBuffer();
  int row = Item_selected_row;
  int col = Item_selected_column;

  u8g2.setFont(u8g2_font_helvR14_tr);
  u8g2.drawStr(0, 17, "Porte:");
  if(row == 1 && col == 1){
    u8g2.drawButtonUTF8(60,17, U8G2_BTN_INV, 0, 1, 1, "Ouvrir");
  }else{
    u8g2.drawButtonUTF8(60,17, U8G2_BTN_BW1, 0, 0, 0, "Ouvrir");
  }
  if(row == 1 && col == 2){
    u8g2.drawButtonUTF8(120,17, U8G2_BTN_INV, 0, 1, 1, "Fermer");
  }else{
    u8g2.drawButtonUTF8(120,17, U8G2_BTN_BW1, 0, 0, 0, "Fermer");
  }
  u8g2.drawStr(0, 38, "Distribution:");
  if(row == 2 && col == 1){
    u8g2.drawButtonUTF8(108,38, U8G2_BTN_INV, 0, 1, 1, "Delivrer");
  }else{
    u8g2.drawButtonUTF8(108,38, U8G2_BTN_BW1, 0, 0, 0, "Delivrer");
  }
  u8g2.drawStr(0, 59, "Balise:"); 
  if(row == 3 && col == 1){
    u8g2.drawButtonUTF8(65,59, U8G2_BTN_INV, 0, 1, 1, "Ajouter");
  }else{
    u8g2.drawButtonUTF8(65,59, U8G2_BTN_BW1, 0, 0, 0, "Ajouter");
  }
  if(row == 3 && col == 2){
    u8g2.drawButtonUTF8(134,59, U8G2_BTN_INV, 0, 1, 1, "Reset"); 
  }else{
    u8g2.drawButtonUTF8(134,59, U8G2_BTN_BW1, 0, 0, 0, "Reset"); 
  }
  if(row == 4 && col == 1){
    u8g2.drawXBMP(237,47,17,17,return_inv_icon);
  }else{
    u8g2.drawXBMP(237,47,17,17,return_icon);
    u8g2.drawFrame(236,46,20,18);
  }
  
  u8g2.updateDisplayArea(7,0,25,8);
  return;
}

void printPortionsMenuNavigation(){
  u8g2.clearBuffer();
  int row = Item_selected_row;
  int col = Item_selected_column;
  u8g2.setFont(u8g2_font_helvR14_tr);
  u8g2.drawStr(0, 17, "Portion:");
  if(row == 1 && col == 1 || (feedingSelected ==1 && row ==2)){
    u8g2.drawButtonUTF8(70,17, U8G2_BTN_INV, 0, 1, 1, "1");  
  }
  else{
    u8g2.drawButtonUTF8(70,17, U8G2_BTN_BW1, 0, 0, 0, "1");  
  }
  if(row == 1 && col == 2 || (feedingSelected ==2 && row == 2)){
    u8g2.drawButtonUTF8(90,17, U8G2_BTN_INV, 0, 1, 1, "2");  
  }
  else{
    u8g2.drawButtonUTF8(90,17, U8G2_BTN_BW1, 0, 0, 0, "2");  
  } 
  u8g2.drawStr(0, 50, "Heure:");
  if(feedingSelected == 1){
    u8g2.drawStr(60,50, timeFeeding1.c_str());
  }
  else if (feedingSelected==2){
    u8g2.drawStr(60,50, timeFeeding2.c_str());
  }
  if(row == 2){ //sélectionner chiffre
    switch (col)
    {
    case 1:
      u8g2.drawLine(61,52,68,52);
      break;
    case 2:
      u8g2.drawLine(71,52,78,52);
      break;
    case 3:
      u8g2.drawLine(86,52,93,52);
      break;
    case 4:
      u8g2.drawLine(96,52,103,52);
      break;
    default:
      break;
    }
  }
  if(row == 3 && col == 1){
    u8g2.drawXBMP(237,47,17,17,return_inv_icon);
  }else{
    u8g2.drawXBMP(237,47,17,17,return_icon);
    u8g2.drawFrame(236,46,20,18);
  }
  u8g2.sendBuffer();
}

String changeTime(String time, bool direction){
  char charHourMS = time.charAt(0);
  char charHourLS = time.charAt(1);
  char charMinutesMS = time.charAt(3);
  char charMinuteLS = time.charAt(4);
  if(direction == 1){
    switch (Item_selected_column)
    {
      case 1:
        if(charHourMS < '2'){
          time[0] = charHourMS + 1;
        }
        break;
      case 2:
        if(charHourLS < '9'){
          time[1] = charHourLS + 1;
        }
        break;
      case 3:
        if(charMinutesMS < '5'){
          time[3] = charMinutesMS + 1;
        }
        break;
      case 4:
        if(charMinuteLS < '9'){
          time[4] = charMinuteLS + 1;
        }
        break;
      default:
        break;
    }
  }
  else{
    switch (Item_selected_column)
    {
      case 1:
        if(charHourMS > '0'){
          time[0] = charHourMS - 1;
        }
        break;
      case 2:
        if(charHourLS > '0'){
          time[1] = charHourLS - 1;
        }
        break;
      case 3:
        if(charMinutesMS > '0'){
          time[3] = charMinutesMS - 1;
        }
        break;
      case 4:
        if(charMinuteLS > '0'){
          time[4] = charMinuteLS - 1;
        }
        break;
      default:
        break;
    }
  }

  return time;
}

uint32_t getWeight(){
  //read ADC 6 times
  uint32_t value = 0;
  for(int i = 0; i<6; i++){
    digitalWrite(33, LOW);           // set the SS pin HIGH
    uint8_t cmd = (MAX_CMD_REG | MAX_DATA) | MAX_READ;
    SPI.transfer(cmd);             // send a write command to the MCP4131 to write at registry address 0x00
    uint8_t valMSB = SPI.transfer(0);
    uint8_t valLSB = SPI.transfer(0);
    uint8_t valLLSB = SPI.transfer(0);
    //printf("valMSB : 0x%X\r\n", valMSB);
    //printf("valLSB : 0x%X\r\n", valLSB);
    value += ((valMSB<<16)|(valLSB<<8)|valLLSB)>>8;
    digitalWrite(33, HIGH);           // set the SS pin HIGH
    delay(150);
  }
  value = (float)value/6;

  printf("\r\n");
  printf("valueX : 0x%X\r\n", value);
  printf("value : %d\r\n", value);
  //printf("voltage : %.2fV\r\n", ((float)value/65535)*3.3);
  
  return value;
}

/*Lis et met a jour l'heure à l'aide du RTC*/
void readRTC(){
  tmElements_t tm;

  if (RTC.read(tm)) {
    //Serial.print("Ok, Time = ");
    //Serial.print(return2digits(tm.Hour));
    RTCTimeHour = tm.Hour;
    //Serial.write(':');
    //Serial.print(return2digits(tm.Minute));
    RTCTimeMinutes = tm.Minute;
    //Serial.write(':');
    //Serial.print(return2digits(tm.Second));
    RTCTimeSeconds = tm.Second;
    //Serial.print(", Date (D/M/Y) = ");
    //Serial.print(tm.Day);
    //Serial.write('/');
    //Serial.print(tm.Month);
    //Serial.write('/');
    //Serial.print(tmYearToCalendar(tm.Year));
    //Serial.println();
    hourNow = return2digits(tm.Hour) + ":" + return2digits(tm.Minute);
  } else {
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped.  Please run the SetTime");
      Serial.println("example to initialize the time and begin running.");
      Serial.println();
    } else {
      Serial.println("DS1307 read error!  Please check the circuitry.");
      Serial.println();
    }
  }
}

/*0(off) à 255(on)*/
void setRGB(uint8_t r, uint8_t g, uint8_t b){
  r = (((float)(255-r))/255)*140 + 115; //réduis intensité
  g = (((float)(255-g))/255)*140 + 115;
  b = (((float)(255-b))/255)*140 + 115;
  /*Serial.println(r);
  Serial.println(g);
  Serial.println(b);*/
  analogWrite(PIN_RGB_R, r);
  analogWrite(PIN_RGB_G, g);
  analogWrite(PIN_RGB_B, b);
}

/*lis le capteur de distance, retourne la distance en mm*/
int readDistance(){
  if (sensor.timeoutOccurred()) { 
    Serial.print(" TIMEOUT"); 
    if (!sensor.init()) //realive
    {
      Serial.println("Failed to detect and initialize sensor!");
    }
    sensor.startContinuous();
  }
  return sensor.readRangeContinuousMillimeters();
}

/*tourne une fois le distributeur*/
void distribute(){
  digitalWrite(PIN_MOTOR_DIST, HIGH);
  while(digitalRead(PIN_SWITCH_DIST) == LOW){} //tourne
  while(digitalRead(PIN_SWITCH_DIST) == HIGH){} //tourne encore
  digitalWrite(PIN_MOTOR_DIST, LOW); //arrete
}

void openDoor(){
  if(!doorOpened){
  myservo.attach(PIN_SERVO);
  for (int pos = 169; pos >= 90; pos -= 1) { 
    myservo.write(pos);             
    delay(20);                       
  }
  myservo.detach();
  doorOpened = true;
  }
}


void closeDoor(){
  if(doorOpened){
  myservo.attach(PIN_SERVO);
  for (int pos = 90; pos <= 169; pos += 1) { 
    // in steps of 1 degree
    myservo.write(pos);              
    delay(20);                      
  }
  myservo.detach();
  doorOpened = false;
  }
}

/*retourne vrai si objet à proximité detécté*/
bool readCapProx(){
  tone(PIN_IN_PROX, 38000);
  delay(200);
  /*Serial.print(digitalRead(PIN_OUT_PROX_1));
  Serial.print(",");
  Serial.println(digitalRead(PIN_OUT_PROX_2));*/
  bool capProx = !(digitalRead(PIN_OUT_PROX_1) || digitalRead(PIN_OUT_PROX_2));
  noTone(PIN_IN_PROX);
  return(capProx);
}


void setup()
{
  //----------------------------------------------------MONITEUR SÉRIE
  Serial.begin(115200);

  //----------------------------------------------------RGB
  pinMode(PIN_RGB_R, OUTPUT);
  digitalWrite(PIN_RGB_R, HIGH);
  pinMode(PIN_RGB_G, OUTPUT);
  digitalWrite(PIN_RGB_G, HIGH);
  pinMode(PIN_RGB_B, OUTPUT);
  digitalWrite(PIN_RGB_B, HIGH);
  setRGB(0,255,0); //vert

  //----------------------------------------------------ÉCRAN
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();
  u8g2.drawXBMP(76,0,103,64,logo);
  u8g2.sendBuffer();

  //----------------------------------------------------Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  int nbTries = 0;
  while (WiFi.status() != WL_CONNECTED && nbTries<10) { //timeout
    delay(500);
    Serial.print(".");
    nbTries++;
  }
  if(WiFi.status() == WL_CONNECTED){ //imprime info si connecté
    Serial.println("");
    Serial.println("WiFi connected.");
    wifiConnected = true;
    Serial.println(WiFi.localIP());
  }

  //----------------------------------------------------Time & RTC
  if(WiFi.status() == WL_CONNECTED){
    configTime(gmtOffset_sec, Offset_sec, ntpServer); //récupère l'heure d'internet
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
    }else{
      RTCTimeHour = timeinfo.tm_hour;
      RTCTimeMinutes = timeinfo.tm_min;
      RTCTimeSeconds = timeinfo.tm_sec;
    }
    Serial.print(RTCTimeHour);
    Serial.print(RTCTimeMinutes);
    Serial.println(RTCTimeSeconds);

    tm.Hour = RTCTimeHour;
    tm.Minute = RTCTimeMinutes;
    tm.Second = RTCTimeSeconds;

    bool config=false;
    if (RTC.write(tm)) { //set le RTC
      config = true;
    }
    
    if (config) {
      Serial.print("DS1307 configured Hour=");
      Serial.print(RTCTimeHour);
      Serial.print(", Minutes=");
      Serial.println(RTCTimeMinutes);
    } else {
      Serial.println("DS1307 Communication Error :-{");
      Serial.println("Please check your circuitry");
      criticalError = true;
    }
  }

  //----------------------------------------------------Balance                      //délais??????????
  pinMode(PIN_CS_ADC_SCALE, OUTPUT); // set the SS pin as an output
  //SPI.begin();

  digitalWrite(PIN_CS_ADC_SCALE, LOW);            // set the SS pin to LOW
  uint8_t cmds = 0x90; //0x48
  SPI.transfer(cmds);
  digitalWrite(PIN_CS_ADC_SCALE, HIGH); 

  delay(1000);

  //Buffer and unipolar
  digitalWrite(PIN_CS_ADC_SCALE, LOW);
  uint8_t cmd = (MAX_CMD_REG | MAX_CTRL1) | MAX_WRITE;
  uint8_t ctrl = 0b01011000; 
  SPI.transfer(cmd);
  SPI.transfer(ctrl);
  digitalWrite(PIN_CS_ADC_SCALE, HIGH);

  delay(500);

  //gain and Self-calibration
  digitalWrite(PIN_CS_ADC_SCALE, LOW);
  uint8_t cmdg = (MAX_CMD_REG | MAX_CTRL3) | MAX_WRITE;
  uint8_t ctrlg = 0b11111000; 
  SPI.transfer(cmdg);
  SPI.transfer(ctrlg);
  digitalWrite(PIN_CS_ADC_SCALE, HIGH);

  delay(500);

  //Conv
  digitalWrite(PIN_CS_ADC_SCALE, LOW);            // set the SS pin to LOW
  uint8_t cmd0 = MAX_CMD_CONV | MAX_SPS_10; //rate 0 (base)
  SPI.transfer(cmd0);
  digitalWrite(PIN_CS_ADC_SCALE, HIGH); 
  delay(500);
  //Make sure its fine by reading CTRL1
  digitalWrite(PIN_CS_ADC_SCALE, LOW);           // set the SS pin HIGH
  uint8_t cmd2 = (MAX_CMD_REG | MAX_CTRL1) | MAX_READ;
  SPI.transfer(cmd2);             // send a write command to the MCP4131 to write at registry address 0x00
  uint8_t val = SPI.transfer(0);
  Serial.println("val : " + (String)val);
  digitalWrite(PIN_CS_ADC_SCALE, HIGH);           // set the SS pin HIGH

  //----------------------------------------------------Timer
  timer = timerBegin(0, 80, true);             // Begin timer with 1 MHz frequency (80MHz/80)
  timerAttachInterrupt(timer, &onTimer, true); // Attach the interrupt to Timer1
  unsigned int timerFactor = 500000;           //500ms
  timerAlarmWrite(timer, timerFactor, true);   // Initialize the timer
  timerAlarmEnable(timer);

  //-----------------------------------------------------BOUTONS
  pinMode(PIN_BUT_UP, INPUT);
  attachInterrupt(PIN_BUT_UP, keypadUPInterrupt, FALLING);
  pinMode(PIN_BUT_DOWN, INPUT);
  attachInterrupt(PIN_BUT_DOWN, keypadDOWNInterrupt, FALLING);
  pinMode(PIN_BUT_LEFT, INPUT);
  attachInterrupt(PIN_BUT_LEFT, keypadLEFTInterrupt, FALLING);
  pinMode(PIN_BUT_RIGHT, INPUT);
  attachInterrupt(PIN_BUT_RIGHT, keypadRIGHTInterrupt, FALLING);
  pinMode(PIN_BUT_SELECT, INPUT);
  attachInterrupt(PIN_BUT_SELECT, keypadSELECTInterrupt, FALLING);

  //-----------------------------------------------------BATTERIE
  pinMode(PIN_BATT_MON, INPUT);
  batDis = (float)analogRead(PIN_BATT_MON) / 4.0 ; //niveau batterie distributeur 0 à 255

  //-----------------------------------------------------Servo
  //ESP32PWM::allocateTimer(4);
  myservo.setPeriodHertz(50);      // standard 50 hz servo

  //-----------------------------------------------------Capteur de distance
  sensor.setTimeout(500);
  //sensor.setTimeout(0);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor! Lets try once more...");
      if (!sensor.init())
    {
    Serial.println("Failed to detect and initialize sensor! OOPS");
    criticalError = true;
    }
  }
  sensor.startContinuous();

  //-----------------------------------------------------Capteur de proximité
  pinMode(PIN_OUT_PROX_1, INPUT);
  pinMode(PIN_OUT_PROX_2, INPUT);
  pinMode(PIN_IN_PROX, OUTPUT);

  //-----------------------------------------------------Distribution
  pinMode(PIN_MOTOR_DIST, OUTPUT);
  digitalWrite(PIN_MOTOR_DIST, LOW);
  pinMode(PIN_SWITCH_DIST, INPUT);

  //-----------------------------------------------------IR
  IrReceiver.begin(PIN_IR_RX); // Initializes the IR receiver object
  pinMode(PIN_IR_TX, OUTPUT);

  //-----------------------------------------------------SETUP FINI
  if(criticalError){ //Erreur critique... rien à faire...
    setRGB(255,0,0);
    while(true){};
  }

  //lire RTC
  readRTC();

  //imprime menu principal
  menu_selected = MAIN_MENU;
  printMenu(menu_selected);

  //ferme rgb
  setRGB(0,0,0);

}


void loop()
{

  //Comm IR
  if(!doorOpened || stateComm == CAT_EATING){
    switch (stateComm){
    case STANDBY :
    {
      int dist = readDistance();
      if (dist < distanceMin && dist > 50){ //présence détecté
        setRGB(255,255,0); //jaune     
        stateComm = TX_COMM;        
        counterTXUptime = 0; 
        tone(PIN_IR_TX, 50); //DEL IR TX 50hz 
      }
      break;
    }
    case TX_COMM :
    {
      if(counterTXUptime >= 1){ //500ms se sont passé
        setRGB(255,255,255); //blanc    
        stateComm = RX_COMM;        
        counterRXUptime = 0; 
        noTone(PIN_IR_TX); //ferme DEL IR
      }
      break;
    }
    case RX_COMM :
    {
      if(counterRXUptime >=3){ //timeout 3 seconde
        setRGB(0,0,0); //ferme RGB    
        stateComm = STANDBY;
        IrReceiver.resume(); // Important, enables to receive the next IR signal
      }
      if(IrReceiver.decode()) { //décode IR recu de la balise
        Serial.println("Received something...");    
        IrReceiver.printIRResultShort(&Serial); // imprime donnée IR recu
        Serial.println();
        if((IrReceiver.decodedIRData.protocol == NEC || IrReceiver.decodedIRData.protocol == NEC2) && IrReceiver.decodedIRData.address==0x12){ //check le protocole et adresse envoyé
          Serial.println("Tag code received!");
          setRGB(148,0,211); //flash mauve
          openDoor();
          doorOpenedByCat = true;
          counterDoorOpenCatLeft = 0;
          stateComm = CAT_EATING;
          setRGB(0,0,0); //ferme rgb
        }
        IrReceiver.resume(); // Important, enables to receive the next IR signal
      }
      break;
    }


    case CAT_EATING :
    {
      bool prox = readCapProx();
      if(prox){ //chat encore la
        counterDoorOpenCatLeft = 0;
        setRGB(100,100,0);
      }
      Serial.println(prox);
      if(counterDoorOpenCatLeft>=5){ //chat parti depuis 5 seconde
        closeDoor();
        setRGB(0,0,0);
        doorOpenedByCat = false;
        stateComm = STANDBY;
      }
      break;
    }


    default :
    {
      break;
    }
    }
  }


/*
  //Lancer comm IR
  if(!commToStart && !commStarted && !doorOpened && !doorOpenedByCat){
    //int dist = readDistance();
    //Serial.println(dist);
    if (dist < distanceMin && dist > 50){
      commToStart = true;
    }
  }*/

  //Comm IR
  /*if (commToStart){ //débute communication IR au 3 seconde
    Serial.println("comm started");
    setRGB(255,255,0); //jaune
    tone(PIN_IR_TX, 49); //TX 50hz
    counterTXUptime = 0;
    commToStart = false;   
    TXCommStarted = true;
    IRTXup = true;
  }
  else if(TXCommStarted){ //communication IR en cours
    if(counterTXUptime >= 1 && IRTXup){ //après 500ms
      noTone(PIN_IR_TX); //ferme TX
      setRGB(255,255,255); //blanc
      commListen = true;
      counterRXUptime = 0;
      Serial.println("leds closed");
      IRTXup = false;
    }
    if(commListen && counterRXUptime > 3){ //timeout 3 seconde
      commListen = false;
      TXCommStarted = false;
      setRGB(0,0,0); //ferme rgb
      IrReceiver.resume(); // Important, enables to receive the next IR signal
    }
    else if (IrReceiver.decode() && !IRTXup && commListen) { //décode IR recu de la balise
      Serial.println("Received something...");    
      IrReceiver.printIRResultShort(&Serial); // imprime donnée IR recu
      Serial.println();
      if((IrReceiver.decodedIRData.protocol == NEC || IrReceiver.decodedIRData.protocol == NEC2) && IrReceiver.decodedIRData.address==0x12){ //check le protocole et adresse envoyé
        //openDoor = true;
        Serial.println("Tag code received!");
        setRGB(148,0,211); //flash mauve
        openDoor();
        doorOpenedByCat = true;
        counterDoorOpenCatLeft = 0;
        TXCommStarted = false;
        commListen = false;
        setRGB(0,0,0); //ferme rgb
      }
    IrReceiver.resume(); // Important, enables to receive the next IR signal
    }
  }

  if(doorOpenedByCat){ //ferme porte si chat parti
    if(readCapProx()){ //chat encore la
      counterDoorOpenCatLeft = 0;
      setRGB(100,100,0);
    }
    if(counterDoorOpenCatLeft>3){ //chat parti depuis 3 seconde
      closeDoor();
      setRGB(0,0,0);
      doorOpenedByCat = false;
    }
  }*/

  //Heure RTC
  readRTC();

  
  switch(menu_selected){
    case NAVIGATION_MENU :
      if(keypad_right){
        if(Item_selected_column >=1 && Item_selected_column<=3){
          printNavigationMenuBorders(Item_selected_column, Item_selected_column+1);
          Item_selected_column++;
        }
        keypad_right = false;
      }
      else if(keypad_left){
        if(Item_selected_column >=2 && Item_selected_column<=4){
          printNavigationMenuBorders(Item_selected_column, Item_selected_column-1);
          Item_selected_column--;
        }        
        keypad_left = false;
      }
      else if(keypad_select){
        switch(Item_selected_column){
          case 1:
            menu_selected = MAIN_MENU;
            printMenu(menu_selected);
            break;
          case 2:
            menu_selected = PORTIONS_MENU;
            printMenu(menu_selected);
            break;
          case 3:
            menu_selected = HOUR_MENU;
            printMenu(menu_selected);
            break;
          case 4:
            menu_selected = ACTIONS_MENU;
            printMenu(menu_selected);
            break;
        }
        Item_selected_column = 1;
        keypad_select = false;
      }
      break;
      
    case MAIN_MENU : 
      if(RTCTimeMinutes != lastMinutes){
        printMenu(MAIN_MENU); //rafraichi la page (chg de minute)
      }
      if(keypad_select){
        menu_selected = NAVIGATION_MENU;
        printMenu(NAVIGATION_MENU);
        keypad_select = false;
      }
      break;

    case PORTIONS_MENU :
      //navigation
      if(keypad_right){
        if(Item_selected_row == 1 && Item_selected_column == 1){
          Item_selected_column ++;
          printPortionsMenuNavigation();
        }
        else if(Item_selected_row == 1 && Item_selected_column == 2){ //vers bouton retour
          Item_selected_row = 3;
          Item_selected_column = 1;
          printPortionsMenuNavigation();
        }
        keypad_right = false;
      }
      if(keypad_left){
        if(Item_selected_row == 1 && Item_selected_column == 2){
          Item_selected_column --;
          printPortionsMenuNavigation();
        }
        if(Item_selected_row == 3){
          Item_selected_row = 1;
          Item_selected_column = 2;
          printPortionsMenuNavigation();
        }
        keypad_left = false;
      }
      if(keypad_up){
        if(Item_selected_row == 2){
          if(feedingSelected == 1){
            timeFeeding1 = changeTime(timeFeeding1, 1);
          }
          else if (feedingSelected == 2){
            timeFeeding2 = changeTime(timeFeeding2, 1);
          }
        }
        printPortionsMenuNavigation();
        keypad_up = false;
      }
      if(keypad_down){
        if(Item_selected_row == 2){
          if(feedingSelected == 1){
            timeFeeding1 = changeTime(timeFeeding1, 0);
          }
          else if (feedingSelected == 2){
            timeFeeding2 = changeTime(timeFeeding2, 0);
          }
        }
        printPortionsMenuNavigation();
        keypad_down = false;
      }
      //sélection
      if(keypad_select){
        if(Item_selected_row == 3 && Item_selected_column == 1){ //retour menu navigation
          menu_selected = NAVIGATION_MENU;
          printMenu(NAVIGATION_MENU);
          Item_selected_column = 1;
          Item_selected_row = 1;
          keypad_select = false;
          break;
        }
        if(Item_selected_row == 1 && Item_selected_column == 1){ //portion 1
          Item_selected_row = 2;
          Item_selected_column = 1;
          feedingSelected = 1;
          printPortionsMenuNavigation();
          keypad_select = false;
          break;
        }
        if(Item_selected_row == 1 && Item_selected_column == 2){ //portion 2
          Item_selected_row = 2;
          Item_selected_column = 1;
          feedingSelected = 2;
          printPortionsMenuNavigation();
          keypad_select = false;
          break;
        }
        if(Item_selected_row == 2){ //passe de chiffre en chiffre dans l'heure
          if(Item_selected_column <4){
            Item_selected_column++;
            printPortionsMenuNavigation();
          }else{
            Item_selected_row = 1;
            Item_selected_column = feedingSelected;
            printPortionsMenuNavigation();
          }
        }
        keypad_select = false;
      }

      break;

    case ACTIONS_MENU :
      //navigation
      if(keypad_up){
        if(Item_selected_row >= 2){
          Item_selected_row--;
          if(Item_selected_row == 2 || Item_selected_row == 4){ //remettre colomne a 1
            Item_selected_column = 1;
          }
          printActionsMenuButtons();
        }
        keypad_up = false;
      }
      if(keypad_down){
        if(Item_selected_row <= 3){
          Item_selected_row++;
          if(Item_selected_row == 2 || Item_selected_row == 4){ //remettre colomne a 1
            Item_selected_column = 1;
          }
          printActionsMenuButtons();
        }
        keypad_down = false;
      }
      if(keypad_left){
        if(Item_selected_column==2 && (Item_selected_row == 1 || Item_selected_row == 3)){
          Item_selected_column--;
          printActionsMenuButtons();
        }
        if(Item_selected_column==1 && Item_selected_row == 4){ //retour sur ligne 3 et ligne 4
          Item_selected_column = 2;
          Item_selected_row = 3;
          printActionsMenuButtons(); 
        }
        keypad_left = false;
      }
      if(keypad_right){
        if(Item_selected_column==1 && (Item_selected_row == 1 || Item_selected_row == 3)){
          Item_selected_column++;
          printActionsMenuButtons(); 
        }
        else if(Item_selected_column==2 && Item_selected_row == 3){ //retour sur ligne 3 et ligne 4
          Item_selected_column = 1;
          Item_selected_row = 4;
          printActionsMenuButtons(); 
        }
        keypad_right = false;
      }
      //sélection
      if(keypad_select){
        if(Item_selected_row == 1 && Item_selected_column == 1){ //ouvrir porte
          openDoor();
          doorOpenedByManually = true;
          Serial.println("door opened manually");
        }
        if(Item_selected_row == 1 && Item_selected_column == 2){ //fermer porte
          closeDoor();
          doorOpenedByManually = false;
          Serial.println("door closed manually");
        }
        if(Item_selected_row == 2){ //délivrer portion
          distribute();
          Serial.println("delivery done");
        }
        if(Item_selected_row == 3 && Item_selected_column == 1){ //ajouter balise

        }
        if(Item_selected_row == 3 && Item_selected_column == 2){ //reset balise

        }
        if(Item_selected_row == 4 && Item_selected_column == 1){ //retour menu navigation
          menu_selected = NAVIGATION_MENU;
          printMenu(NAVIGATION_MENU);
          Item_selected_column = 1;
          Item_selected_row = 1;
        }
        keypad_select = false;
      }
      break;

    default : 
      break;
  }


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
    counterTimeDoorOpen = 0;
    doorOpened = true;
    u8g2.clearBuffer();                    // clear the internal memory
    u8g2.drawStr(10, 20, "Door open");         // write something to the internal memory
    u8g2.sendBuffer();
    openDoor = false;
  }

  if(doorOpened){
    Serial.println("counter dooropen : " + (String)counterTimeDoorOpen);
  }
  if(doorOpened && counterTimeDoorOpen>10 && distance >= distanceMin){ //ferme la porte après 10 seconde si il ny a plus personne
      myservo.write(0);
      doorOpened = false;
      u8g2.clearBuffer();                    // clear the internal memory
      u8g2.drawStr(10, 20, "Door close");         // write something to the internal memory
      u8g2.sendBuffer();
  }
  else if(doorOpened && counterTimeDoorOpen>10 && distance <= distanceMin){ //quelqun mange toujours
    counterTimeDoorOpen = 0;
  }*/

  /*if(measureDistance){ //mesure distance au sec
    VL53LX_MultiRangingData_t measurement = vl53lxWrapper->getLatestMeasurement();
    distance = measurement.RangeData->RangeMilliMeter;
    Serial.println(distance);
    measureDistance = false;
  }*/
/*
  if(distance <= distanceMin){ //Présence détecté : allume DEL IR
    commStarted = true;
    counterTxUptime = 0;
    digitalWrite(PIN_ACTI, HIGH);
  }
  if(counterTxUptime>2 && commStarted == true){ //Fini de flasher la LED IR pour 1 sec
    digitalWrite(PIN_ACTI, LOW);
    commStarted = false;
    counterTxUptime = 0;
  }*/

}