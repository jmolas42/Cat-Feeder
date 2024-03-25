/*PINS*/

//Batterie
#define PIN_BATT_MON 14      //lecture voltage batterie

//Boutons
#define PIN_BUT_UP 42        //bouton haut
#define PIN_BUT_DOWN 2       //bouton bas
#define PIN_BUT_LEFT 40      //bouton gauche
#define PIN_BUT_RIGHT 1      //bouton droit
#define PIN_BUT_SELECT 3     //bouton sélection

//Écran
#define PIN_DC_SCREEN 20     //DC écran
#define PIN_CS_SCREEN 19     //chip select écran
#define PIN_RESET_SCREEN 34  //reset écran

//RGB
#define PIN_RGB_R 41         //RGB rouge
#define PIN_RGB_G 38         //RGB vert
#define PIN_RGB_B 39         //RGB bleu

//IR
#define PIN_IR_RX 13         //photorécepteur
#define PIN_IR_TX 17         //DEL IR

//Capteur de proximité
#define PIN_OUT_PROX_1 12    //photorécepteur capteur de proximité 1
#define PIN_OUT_PROX_2 7     //photorécepteur capteur de proximité 2
#define PIN_IN_PROX 4        //DEL capteurs de proximité (même signal pour les deux)

//Capteur de distance
#define PIN_INT_DIST 6       //interrupt capteur de distance
//Comm série
#define PIN_SDA 8            //SDA
#define PIN_SCL 9            //SCL
#define PIN_MOSI 35          //MOSI
#define PIN_MISO 37          //MISO
#define PIN_SCK 36           //SCK
//ADC balance
#define PIN_CS_ADC_SCALE 33  //chip select adc différentiel
//RTC
#define PIN_INT_RTC 10       //interrupt RTC
//Servo porte
#define PIN_SERVO 5          //servomoteur porte
//Distribution
#define PIN_MOTOR_DIST 15    //activation moteur distribution
#define PIN_SWITCH_DIST 16   //rétroaction moteur distribution