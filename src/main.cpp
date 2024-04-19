#include <Arduino.h>
#include <IRremote.h> 
#include <ESP32Servo.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <pins.h>
#include <svg.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <DS1337RTC.h>
#include <Time.h>
#include <Wire.h>
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include <Preferences.h>
#include <WiFiUdp.h>
#include <SPIFFS.h>
#include <VL53L0X.h>

//flags systeme
bool criticalError = false;

//flash
Preferences prefs;

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
int weight = 0;
float const calibration = 10.436;  //bits par gramme

//WiFi
bool wifiConnected = false;
String ssid     = "Test";
String password = "test1234";
DNSServer dnsServer;
AsyncWebServer server(80);
bool WifiCredentialsSet = false;
bool WifiSTA = true; //configuration normale ou sinon AP pour la config
bool upANDdownActivated = false; //si bouton hautet bas sont enfoncé
int timerupANDdownActivated = 0;
bool startBroadcast = false;

//Time & RTC
String hourNow = "00:00";
int RTCTimeHour = 0;
int RTCTimeMinutes =  0;
int RTCTimeSeconds = 0; 
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   Offset_sec = -14400; //-4h
tmElements_t tm;
bool alarmRTC = false;
bool updateTimeFlag = false;

//capteur de distance
VL53L0X sensor;
const int distanceMin = 325;
int distances[3] = {0,0,0};
uint8_t distancesIndex = 0;

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
int millisCommIR = 0;
int tagID = 0x12;

//Portions
int lastFeeding = 1;
int feedingPortions [2] = {0,0};
String timeFeeding1 = "08:00";
int nbFeeding1 = 0;
String timeFeeding2 = "17:00";
int nbFeeding2 = 0;

//Batteries
uint8_t batTag = 0; //batterie balise 0 à 255
uint8_t batDis = 0; //batterie distributeur 0 à 255
bool lowBattery = false;
bool readBattery = false;
int timerLowBattery = 0; //timer RGB flash rouge si batterie presque vide

//Interface utilisateur
U8G2_SSD1322_NHD_256X64_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/PIN_SCK, /* data=*/PIN_MOSI, /* cs=*/PIN_CS_SCREEN, /* dc=*/PIN_DC_SCREEN, /* reset=*/PIN_RESET_SCREEN); // Enable U8G2_16BIT in u8g2.h
enum Menus  {SCREEN_OFF, NAVIGATION_MENU, MAIN_MENU, PORTIONS_MENU, HOUR_MENU, ACTIONS_MENU};
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
int timerScreenNoActivity = 0;
bool screenON = false;
int powerLEDTimer = 0;
bool powerLEDON = true;

uint8_t RGB_R = 0;
uint8_t RGB_G = 0;
uint8_t RGB_B = 0;
uint8_t RGB_R_Last = 0;
uint8_t RGB_G_Last = 0;
uint8_t RGB_B_Last = 0;


// Timer
hw_timer_t *timer = NULL;
volatile bool interruptbool1 = false;
int timerSecond =  0; //pair si timer tombe sur 1 seconde

/*0(off) à 255(on)*/
void setRGB(uint8_t r, uint8_t g, uint8_t b){
  RGB_R_Last = RGB_R;
  RGB_G_Last = RGB_G;
  RGB_B_Last = RGB_B;
  RGB_R = r;
  RGB_G = g;
  RGB_B = b;
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

/*remets la RGB à ce qu'elle était avant*/
void setRGBlast(){
  uint8_t r = (((float)(255-RGB_R_Last))/255)*140 + 115; //réduis intensité
  uint8_t g = (((float)(255-RGB_G_Last))/255)*140 + 115;
  uint8_t b = (((float)(255-RGB_B_Last))/255)*140 + 115;
  analogWrite(PIN_RGB_R, r);
  analogWrite(PIN_RGB_G, g);
  analogWrite(PIN_RGB_B, b);
}

void IRAM_ATTR onTimer()  //compteur de 500ms
{
  interruptbool1 = true; // Indicates that the interrupt has been entered since the last time its value was changed to false

  if(WifiSTA){ //mdde normal
    if(timerSecond % 60 == 0){ //1 minute
      readBattery = true;
    }
    if(timerSecond % 2 == 0){ //1 seconde
      if(upANDdownActivated){
        timerupANDdownActivated++;
      }
      if(stateComm == RX_COMM){
        counterRXUptime++;
      }
      if(doorOpenedByCat){
        counterDoorOpenCatLeft++;
      }
      updateTimeFlag = true;
      if(screenON){
        timerScreenNoActivity++;
      }
      if(powerLEDON){ //ferme led après 5 sec
        powerLEDTimer ++ ;
        if(powerLEDTimer == 5){
          setRGB(0,0,0);
        }
      }
      if(lowBattery){
        timerLowBattery++;
      }
    }
    if(stateComm == TX_COMM){
      counterTXUptime++;
    }
    timerSecond++;
  }
}

void keypadUPInterrupt(){
  if((millis() - lastDebounceTime) > debounceDelay && (menu_selected == HOUR_MENU || menu_selected == ACTIONS_MENU || menu_selected == PORTIONS_MENU)){
    keypad_up = true;
    lastDebounceTime = millis();
    timerScreenNoActivity = 0;
  }
}

void keypadDOWNInterrupt(){
  if((millis() - lastDebounceTime) > debounceDelay && (menu_selected == HOUR_MENU || menu_selected == ACTIONS_MENU || menu_selected == PORTIONS_MENU)){
    keypad_down = true;
    lastDebounceTime = millis();
    timerScreenNoActivity = 0;
  }
}

void keypadLEFTInterrupt(){
  if((millis() - lastDebounceTime) > debounceDelay && (menu_selected == HOUR_MENU || menu_selected == NAVIGATION_MENU || menu_selected == ACTIONS_MENU || menu_selected == PORTIONS_MENU)){
    keypad_left = true;
    lastDebounceTime = millis();
    timerScreenNoActivity = 0;
  }
}

void keypadRIGHTInterrupt(){
  if((millis() - lastDebounceTime) > debounceDelay && (menu_selected == HOUR_MENU || menu_selected == NAVIGATION_MENU || menu_selected == ACTIONS_MENU || menu_selected == PORTIONS_MENU)){
    keypad_right = true;
    lastDebounceTime = millis();
    timerScreenNoActivity = 0;
  }
}

void keypadSELECTInterrupt(){
  if((millis() - lastDebounceTime) > debounceDelay){
    keypad_select = true;
    lastDebounceTime = millis();
    timerScreenNoActivity = 0;
  }
}

void alarmRTCInterrupt(){
  alarmRTC = true;
}


class CaptiveRequestHandler : public AsyncWebHandler
{
public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest *request)
  {
    // request->addInterestingHeader("ANY");
    return true;
  }

  void handleRequest(AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/index.html", "text/html");
  }
};

void setupServer()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", "text/html"); 
            Serial.println("Client Connected"); });

  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      String inputMessage;
      String inputParam;
  
      if (request->hasParam("ssid")) {
        inputMessage = request->getParam("ssid")->value();
        inputParam = "ssid";
        ssid = inputMessage;
        prefs.putString("ssid", inputMessage);
        Serial.println(inputMessage);
      }

      if (request->hasParam("password")) {
        inputMessage = request->getParam("password")->value();
        inputParam = "password";
        password = inputMessage;
        prefs.putString("password", inputMessage);
        Serial.println(inputMessage);
      }
      request->send(200, "text/html", "The values entered by you have been successfully sent to the device <br><a href=\"/\">Return to Home Page</a>");
      WifiCredentialsSet = true; });
}

void connectWiFi()
{
  WiFi.persistent(false);
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.persistent(true);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid.c_str(), password.c_str());
  int nbTries = 0;
  while (WiFi.status() != WL_CONNECTED && nbTries<15) { //timeout
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
  else{
    Serial.println("Unable to connect to WiFi");
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
      if(batDis <=5){ //0
        u8g2.drawXBMP(45,45,25,25,battery_empty_icon);
      }
      else if(batDis > 5 && batDis <= 30){ //15
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
      u8g2.drawStr(120, 50, "Nb:");
      u8g2.drawStr(155, 50, ((String)nbFeeding1).c_str());
      u8g2.drawXBMP(237,47,17,17,return_icon);
      u8g2.drawFrame(236,46,20,18); 
      break;

    case HOUR_MENU:
      u8g2.setFont(u8g2_font_helvR14_tr);
      u8g2.drawStr(0, 17, "Fuseau horaire:");
      u8g2.drawStr(137, 17, "-5");
      u8g2.drawStr(0, 50, "Heure:");
      u8g2.drawStr(60, 50, hourNow.c_str());
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
  u8g2.drawStr(120, 50, "Nb:");

  if((feedingSelected == 1 && row == 2) || (row == 1 && col == 1)){
    u8g2.drawStr(60,50, timeFeeding1.c_str());
    u8g2.drawStr(155, 50, ((String)nbFeeding1).c_str());

  }
  else if ((feedingSelected == 2 && row == 2) || (row == 1 && col == 2) || (row == 3)){
    u8g2.drawStr(60,50, timeFeeding2.c_str());
    u8g2.drawStr(155, 50, ((String)nbFeeding2).c_str()); 
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
    case 5:
      u8g2.drawLine(156,52,162,52);
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

/*retourne un string de temps HH:MM incrémenter ou décrementer*/
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
        if(charHourMS != '2' && charHourLS < '9'){
          time[1] = charHourLS + 1;
        }
        else if(charHourMS == '2' && charHourLS < '4'){ //limite à 24
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

  if(time[0] == '2' && charHourLS > '4'){ //limite à 24h
    time[1] = '4';
  }

  return time;
}

uint32_t getWeight(){
  SPI.begin();
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
  SPI.end();
  u8g2.initInterface();
  if(screenON){
    u8g2.setPowerSave(0);
  }
  return value;
}

/*Lis et met a jour l'heure à l'aide du RTC*/
void updateTime(){
  //lis l'heure
  time_t clock = RTC.get(CLOCK_ADDRESS);
  tmElements_t clockSet;
  breakTime(clock, clockSet); //clock => clockset

  //print the time using Time.h
  /*Serial.print((int)clockSet.Day);
  Serial.print(" ");
  Serial.print((int)clockSet.Month);//monthName[month() - 1]);
  Serial.print(" ");
  Serial.print((int)(1970+clockSet.Year));//year());
  Serial.print(" ");
  Serial.print((int)clockSet.Hour);//hour());
  Serial.print(return2digits(clockSet.Minute));//minute());
  Serial.print(return2digits(clockSet.Second));//second());
  Serial.println("");*/

  //mets a jour l'heure
  RTCTimeHour = clockSet.Hour;
  RTCTimeMinutes = clockSet.Minute;
  RTCTimeSeconds = clockSet.Second;
  hourNow = return2digits(clockSet.Hour) + ":" + return2digits(clockSet.Minute);
}

/*lis le capteur de distance, retourne la distance en mm*/
int readDistance(){
  bool ok = false;
  int dist = 0;
  digitalWrite(PIN_XSHUT_DIST, LOW);
  delay(20);
  digitalWrite(PIN_XSHUT_DIST, HIGH);
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
  else{
    sensor.startContinuous();
    dist = sensor.readRangeContinuousMillimeters();
    distances[distancesIndex] = dist;
    if(dist>50 && dist<325){
      int moyenne = (float)(distances[0]+distances[1]+distances[2]) / 3;
      if(dist < moyenne + 20 && dist > moyenne - 20){ //si les 3 derniere distance sont pas trop éloigné
        Serial.print("ALLOWED ");
        ok = true;
        Serial.println(dist);
      }
      else{
        Serial.print("NOT ALLOWED ");
        Serial.println(dist);
      }
      digitalWrite(PIN_XSHUT_DIST, LOW);
    }
  }

  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  if(distancesIndex<2){
    distancesIndex++;
  }
  else{
    distancesIndex = 0;
  }
  if(ok){
    return dist;
  }
  else{
    return 0;
  }
}

/*tourne une fois le distributeur*/
void distribute(){
  digitalWrite(PIN_MOTOR_DIST, HIGH);
  bool next = false;
  while(!next){
    int first = digitalRead(PIN_SWITCH_DIST);
    delay(10);
    int second = digitalRead(PIN_SWITCH_DIST);
    if(first && second){ //deux a high
      next = true;
    }
  }
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
  prefs.putBool("doorOpened", true);
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
  prefs.putBool("doorOpened", false);
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

/*configure la flash et le RTC pour la distribution*/
void updateFeedingParameters(){
  prefs.putString("timeFeeding1", timeFeeding1); //update flash
  prefs.putString("timeFeeding2", timeFeeding2);
  tm.Second = 0;
  tm.Minute = timeFeeding1.substring(3,5).toInt();
  tm.Hour = timeFeeding1.substring(0,2).toInt();
  RTC.set(makeTime(tm), ALARM1_ADDRESS);
  tm.Minute = timeFeeding2.substring(3,5).toInt();
  tm.Hour = timeFeeding2.substring(0,2).toInt();
  RTC.set(makeTime(tm), ALARM2_ADDRESS);
  Serial.println("New feeding parameters...");
  Serial.println("timeFeeding1 : " + timeFeeding1);
  Serial.println("timeFeeding2 : " + timeFeeding2);

  prefs.putInt("nbFeeding1", nbFeeding1);
  prefs.putInt("nbFeeding2", nbFeeding2);
  Serial.println("nbFeeding1 : " + (String)nbFeeding1);
  Serial.println("nbFeeding2 : " + (String)nbFeeding2);
}


void setup()
{
  //----------------------------------------------------MONITEUR SÉRIE
  Serial.begin(115200);
  
  //----------------------------------------------------Timer
  timer = timerBegin(0, 80, true);             // Begin timer with 1 MHz frequency (80MHz/80)
  timerAttachInterrupt(timer, &onTimer, true); // Attach the interrupt to Timer1
  unsigned int timerFactor = 500000;           //500ms
  timerAlarmWrite(timer, timerFactor, true);   // Initialize the timer
  timerAlarmEnable(timer);

  //----------------------------------------------------RGB
  pinMode(PIN_RGB_R, OUTPUT);
  digitalWrite(PIN_RGB_R, HIGH);
  pinMode(PIN_RGB_G, OUTPUT);
  digitalWrite(PIN_RGB_G, HIGH);
  pinMode(PIN_RGB_B, OUTPUT);
  digitalWrite(PIN_RGB_B, HIGH);
  setRGB(0,255,0); //vert
  powerLEDTimer = 0;

  //----------------------------------------------------ÉCRAN
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();
  u8g2.drawXBMP(76,0,103,64,logo);
  u8g2.sendBuffer();

  //----------------------------------------------------SPIFFS
  if (!SPIFFS.begin())
  {
    Serial.println("Erreur SPIFFS...");
    return;
  }

  File root = SPIFFS.open("/");
  File fileSpiffs = root.openNextFile();

  while (fileSpiffs) // Montre tout les fichiers dans la flash
  {
    Serial.print("File: ");
    Serial.println(fileSpiffs.name());
    fileSpiffs.close();
    fileSpiffs = root.openNextFile();
  }

  //----------------------------------------------------FLASH
  Serial.println("Reading flash...");
  prefs.begin("cat-feeder", false);
  ssid = prefs.getString("ssid", "Test");
  Serial.println("SSID : " + ssid);
  password = prefs.getString("password", "test1234");
  Serial.println("password : " + password);
  timeFeeding1 = prefs.getString("timeFeeding1", "08:00");
  Serial.println("timeFeeding1 : " + timeFeeding1);
  nbFeeding1 = prefs.getInt("nbFeeding1", 0);
  Serial.println("nbFeeding1 : " + (String)nbFeeding1);
  timeFeeding2 = prefs.getString("timeFeeding2", "17:00");
  Serial.println("timeFeeding2 : " + timeFeeding2);
  nbFeeding2 = prefs.getInt("nbFeeding2", 0);
  Serial.println("nbFeeding2 : " + (String)nbFeeding2);
  batTag = prefs.getInt("batTag", 0);
  Serial.println("batTag : " + (String)batTag);
  tagID = prefs.getInt("tagID", 0x12);
  Serial.println("tagID : " + (String)tagID);
  doorOpened = prefs.getBool("doorOpened", false);
  Serial.println("doorOpened : " + (String)doorOpened);

  //----------------------------------------------------Wi-Fi
  connectWiFi();

  //----------------------------------------------------Time & RTC
  if(WiFi.status() == WL_CONNECTED){
    configTime(gmtOffset_sec, Offset_sec, ntpServer); //récupère l'heure d'internet
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
    }else{
      tm.Year = timeinfo.tm_year - 70;
      tm.Month = timeinfo.tm_mon + 1;
      tm.Day = timeinfo.tm_mday;
      tm.Hour = timeinfo.tm_hour;
      tm.Minute = timeinfo.tm_min;
      tm.Second = timeinfo.tm_sec;
    }
    Serial.print(tm.Hour);
    Serial.print(tm.Minute);
    Serial.println(tm.Second);

    RTC.set(makeTime(tm), CLOCK_ADDRESS);
    
    Serial.print("DS1337 configured Hour=");
    Serial.print(tm.Hour);
    Serial.print(", Minutes=");
    Serial.println(tm.Minute);
  }
  else{ //pas d'internet => minuit
    tm.Year = 2000 - 1970;
    tm.Month = 2;
    tm.Day = 1;
    tm.Hour = 0;
    tm.Minute = 0;
    tm.Second = 0;
    Serial.print(tm.Hour);
    Serial.print(tm.Minute);
    Serial.println(tm.Second);

    RTC.set(makeTime(tm), CLOCK_ADDRESS);
    Serial.print("DS1337 configured Hour by default=");
    Serial.print(tm.Hour);
    Serial.print(", Minutes=");
    Serial.println(tm.Minute);
  }
  //alarme
  pinMode(PIN_INT_RTC, INPUT);
  digitalWrite(PIN_INT_RTC,HIGH);
  attachInterrupt(PIN_INT_RTC, alarmRTCInterrupt, FALLING);
  tm.Second = 0;
  tm.Minute = timeFeeding1.substring(3,5).toInt();
  tm.Hour = timeFeeding1.substring(0,2).toInt();
  RTC.set(makeTime(tm), ALARM1_ADDRESS);
  tm.Minute = timeFeeding2.substring(3,5).toInt();
  tm.Hour = timeFeeding2.substring(0,2).toInt();
  RTC.set(makeTime(tm), ALARM2_ADDRESS);
  RTC.enableAlarm(ALARM1_ADDRESS);
  RTC.enableAlarm(ALARM2_ADDRESS);
  RTC.freqSelect(1);  // set the squarewave freq on alarm pin b to 4.096kHz

  time_t alarm1 = RTC.get(ALARM1_ADDRESS);  // get the time the alarm is set for
  time_t alarm2 = RTC.get(ALARM2_ADDRESS);  // get the time the alarm is set for
  tmElements_t alarmSet1;
  tmElements_t alarmSet2;
  breakTime(alarm1, alarmSet1);
  breakTime(alarm2, alarmSet2);
  // print the alarm time
  Serial.print("ALARM1: ");
  Serial.print((int)alarmSet1.Hour);
  Serial.print(return2digits(alarmSet1.Minute));
  Serial.print(return2digits(alarmSet1.Second));
  Serial.print("\t");
  // print the alarm time
  Serial.print("ALARM2: ");
  Serial.print((int)alarmSet2.Hour);
  Serial.print(return2digits(alarmSet2.Minute));
  Serial.print(return2digits(alarmSet2.Second));
  Serial.println();

  //----------------------------------------------------Balance                      //délais??????????
  pinMode(33, OUTPUT); // set the SS pin as an output
  SPI.begin();

  digitalWrite(33, LOW);            // set the SS pin to LOW
  uint8_t cmds = 0x90; //0x48
  SPI.transfer(cmds);
  digitalWrite(33, HIGH); 

  delay(1000);

  //Buffer and unipolar
  digitalWrite(33, LOW);
  uint8_t cmd = (MAX_CMD_REG | MAX_CTRL1) | MAX_WRITE;
  uint8_t ctrl = 0b01011000; 
  SPI.transfer(cmd);
  SPI.transfer(ctrl);
  digitalWrite(33, HIGH);

  delay(500);

  //gain and Self-calibration
  digitalWrite(33, LOW);
  uint8_t cmdg = (MAX_CMD_REG | MAX_CTRL3) | MAX_WRITE;
  uint8_t ctrlg = 0b11111000; 
  SPI.transfer(cmdg);
  SPI.transfer(ctrlg);
  digitalWrite(33, HIGH);

  delay(500);


  //Conv
  digitalWrite(33, LOW);            // set the SS pin to LOW
  uint8_t cmd0 = MAX_CMD_CONV | MAX_SPS_10; //rate 0 (base)
  SPI.transfer(cmd0);
  digitalWrite(33, HIGH); 

  delay(500);

  //Make sure its fine by reading CTRL1
  digitalWrite(33, LOW);           // set the SS pin HIGH
  uint8_t cmd2 = (MAX_CMD_REG | MAX_CTRL1) | MAX_READ;
  SPI.transfer(cmd2);             // send a write command to the MCP4131 to write at registry address 0x00
  uint8_t val = SPI.transfer(0);
  Serial.println("val : " + (String)val);
  digitalWrite(33, HIGH);           // set the SS pin HIGH

  delay(50);

  SPI.end();
  u8g2.initInterface();
  u8g2.setPowerSave(0);


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
  Serial.print("batDis: ");
  Serial.println(batDis);

  //-----------------------------------------------------Servo
  //ESP32PWM::allocateTimer(4);
  myservo.setPeriodHertz(50);      // standard 50 hz servo

  //-----------------------------------------------------Capteur de distance
  pinMode(PIN_XSHUT_DIST, OUTPUT); //on essaie un reset du xshut du capteur de distance (résoue bug?)

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

  //porte
  if(doorOpened){
    closeDoor();
  }

  //lire RTC
  updateTime();

  //imprime menu principal
  menu_selected = MAIN_MENU;
  printMenu(menu_selected);
  screenON = true;
  timerScreenNoActivity = 0;

}


void loop()
{
  if(readBattery && WifiSTA){
    batDis = (float)analogRead(PIN_BATT_MON) / 4.0 ; //niveau batterie distributeur 0 à 255
    Serial.print("batDis: ");
    Serial.println(batDis);
    if(batDis < 30){
      lowBattery = true;
    }
    else{
      lowBattery = false;
    }
    readBattery = false;
  }


  if(lowBattery && timerLowBattery >= 300 && !(RGB_R==255 && RGB_G==0 && RGB_B==0)){//allume RGB rouge si batterie vide au 5 minute
    setRGB(255,0,0);
    Serial.println("Open RGB low battery");
  }
  if(lowBattery && timerLowBattery >= 305 && (RGB_R==255 && RGB_G==0 && RGB_B==0)){//5 seconde plus tard on remets la led
    setRGBlast();
    timerLowBattery = 0;
    Serial.println("Close RGB low battery");
  }

  if(digitalRead(PIN_BUT_UP) == LOW && digitalRead(PIN_BUT_DOWN) == LOW){
    if(!upANDdownActivated && !startBroadcast){ //appui débuté
      upANDdownActivated = true;
      timerupANDdownActivated = 0;
    }
    if(upANDdownActivated && timerupANDdownActivated >= 3){ //appui fin 3 sec
      startBroadcast = true;
      upANDdownActivated = false;
    }
  }else if(upANDdownActivated){ //appui arreté avant fin
    upANDdownActivated = false;
  }

  if (startBroadcast && WifiSTA)
  {
    WifiSTA = false;
    Serial.println();
    Serial.println("Setting up AP Mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("CatFeeder_0001", "12345678");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("Setting up Async WebServer");
    setupServer();
    Serial.println("Starting DNS Server");
    dnsServer.start(53, "*", WiFi.softAPIP());
    server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER); // only when requested from AP
    server.begin();
    Serial.println("All Done!");
    u8g2.sleepOff();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvR14_tr);
    u8g2.drawStr(0, 17, "SSID: CatFeeder_0001");
    u8g2.drawStr(0, 50, "mdp: 12345678");
    u8g2.sendBuffer();
    setRGB(255,140,0); //orange
  }

  // Access point
  if (WiFi.getMode() == WIFI_AP)
  {
    dnsServer.processNextRequest();
    if (WifiCredentialsSet || digitalRead(PIN_BUT_SELECT) == LOW)
    {
      u8g2.clearBuffer();
      u8g2.drawXBMP(76, 0, 103, 64, logo);
      u8g2.sendBuffer();
      connectWiFi();
      WifiSTA = true;
      WifiCredentialsSet = false;
      setRGB(0, 0, 0);
      keypad_up = false;
      keypad_down = false;
      keypad_left = false;
      keypad_right = false;
      keypad_select = false;
      startBroadcast= false;
      menu_selected = MAIN_MENU;
      printMenu(menu_selected);
      screenON = true;
      timerScreenNoActivity = 0;
    }
  }

  //Comm IR
  if(WifiSTA && (!doorOpened || stateComm == CAT_EATING)){
    switch (stateComm){
    case STANDBY :
    {
      int dist = readDistance();
      if (dist < distanceMin && dist > 50){ //présence détecté
        Serial.print("object detected at : ");
        Serial.print(dist);
        Serial.println("mm");
        setRGB(255,255,255); //blanc    
        stateComm = TX_COMM;        
        counterTXUptime = 0;
        millisCommIR = millis(); 
        tone(PIN_IR_TX, 50); //DEL IR TX 50hz 
      }
      break;
    }
    case TX_COMM :
    {
      if(millis() > millisCommIR+400){ //400ms se sont passé
        stateComm = RX_COMM;        
        counterRXUptime = 0; 
        noTone(PIN_IR_TX); //ferme DEL IR
        millisCommIR = millis();
      }
      break;
    }
    case RX_COMM :
    {

      if(millis() > millisCommIR + 1000){ //timeout 1 seconde
        setRGB(0,0,0); //ferme RGB    
        stateComm = STANDBY;
        IrReceiver.resume(); // Important, enables to receive the next IR signal
      }
      if(IrReceiver.decode()) { //décode IR recu de la balise
        Serial.println("Received something...");
        if((IrReceiver.decodedIRData.protocol == NEC || IrReceiver.decodedIRData.protocol == NEC2) && IrReceiver.decodedIRData.address == tagID){ //check le protocole et adresse envoyé
          IrReceiver.printIRResultShort(&Serial); // imprime donnée IR recu
          batTag = IrReceiver.decodedIRData.command;
          prefs.putInt("batTag", batTag);
          Serial.println("Tag code received!");
          Serial.print("Battery level Tag : ");
          Serial.println(batTag);
          setRGB(148,0,211); // mauve
          openDoor();
          doorOpenedByCat = true;
          counterDoorOpenCatLeft = 0;
          stateComm = CAT_EATING;
        }
        else{
          Serial.println("It's just noise!");
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
      if(counterDoorOpenCatLeft>=3){ //chat parti depuis 3 seconde
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


  //Heure RTC
  if(WifiSTA && updateTimeFlag){ //update chaque seconde
    updateTime();
    updateTimeFlag = false;
  }
  if(WifiSTA && alarmRTC){
    Serial.println("ALARM!!!!");
    alarmRTC = false;
    RTC.resetAlarms();
    int nbFeeding = 0;
    String h = hourNow.substring(0,2);
    if(h==timeFeeding1.substring(0,2)){ //hours ok
      int mH = hourNow.substring(3,5).toInt(); //check minutes (décalage possible)
      int mF = timeFeeding1.substring(3,5).toInt();
      if((mH == mF) || (mH+1 == mF) || (mH == 59 && mF==00)){
        nbFeeding = nbFeeding1;
      }
    }
    else if(h==timeFeeding2.substring(0,2)){ //hours ok
      int mH = hourNow.substring(3,5).toInt(); //check minutes (décalage possible)
      int mF = timeFeeding2.substring(3,5).toInt();
      if((mH == mF) || (mH+1 == mF) || (mH == 59 && mF==00)){
        nbFeeding = nbFeeding2;
      }
    }
    
    for(int i=0; i<nbFeeding; i++){
      distribute();
    }
  }


  if(WifiSTA && timerScreenNoActivity >30 && screenON){
    Serial.println("Screen turned off");
    u8g2.sleepOn();
    screenON = false;
    menu_selected = SCREEN_OFF;
  }


  if(WifiSTA){
  switch(menu_selected){
    case SCREEN_OFF :
      if(keypad_select){
        Serial.println("Screen turned on");
        u8g2.sleepOff();
        screenON = true;
        timerScreenNoActivity=0;
        menu_selected = MAIN_MENU;
        printMenu(menu_selected);
        keypad_select = false;
      }
      break;
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
        if(Item_selected_row == 2 && Item_selected_column<5){ //heure
          if(feedingSelected == 1){
            timeFeeding1 = changeTime(timeFeeding1, 1);
          }
          else if (feedingSelected == 2){
            timeFeeding2 = changeTime(timeFeeding2, 1);
          }
        }
        if(Item_selected_row == 2 && Item_selected_column==5){ //nb
          if(feedingSelected == 1){
            if(nbFeeding1 < 9){
              nbFeeding1++;
            }
            else{
              nbFeeding1 = 0;
            }
          }
          else if (feedingSelected == 2){
            if(nbFeeding2 < 9){
              nbFeeding2++;
            }
            else{
              nbFeeding2 = 0;
            }
          }
        }

        printPortionsMenuNavigation();
        keypad_up = false;
      }
      if(keypad_down){
        if(Item_selected_row == 2  && Item_selected_column<5){ //heure
          if(feedingSelected == 1){
            timeFeeding1 = changeTime(timeFeeding1, 0);
          }
          else if (feedingSelected == 2){
            timeFeeding2 = changeTime(timeFeeding2, 0);
          }
        }
        if(Item_selected_row == 2 && Item_selected_column==5){ //nb
          if(feedingSelected == 1){
            if(nbFeeding1 > 0){
              nbFeeding1--;
            }
            else{
              nbFeeding1 = 9;
            }
          }
          else if (feedingSelected == 2){
            if(nbFeeding2 > 0){
              nbFeeding2--;
            }
            else{
              nbFeeding2 = 9;
            }
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
          updateFeedingParameters();
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
        if(Item_selected_row == 2){ //passe de chiffre en chiffre dans l'heure et le nombre
          if(Item_selected_column <5){
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

    case HOUR_MENU:
      if(keypad_select){
        menu_selected = NAVIGATION_MENU;
        printMenu(NAVIGATION_MENU);
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
          setRGB(0,0,255);
          Serial.println("door opened manually");
        }
        if(Item_selected_row == 1 && Item_selected_column == 2){ //fermer porte
          closeDoor();
          doorOpenedByManually = false;
          setRGB(0,0,0);
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
  }

}