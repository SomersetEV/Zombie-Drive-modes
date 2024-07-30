// demo: CAN-BUS Shield, receive data with interrupt mode, and set mask and filter
//
// when in interrupt mode, the data coming can't be too fast, must >20ms, or else you can use check mode

#include <SPI.h>


#define CAN_2515
//input buttons
#define ButtonSport 5
#define ButtonEco 4
#define ButtonDrift 3

// output for LEDs
#define LightSport 6
#define LightEco 7
#define LightDrift 8
// #define CAN_2518FD

// Set SPI CS Pin according to your hardware

#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
// For Wio Terminal w/ MCP2518FD RPi Hat：
// Channel 0 SPI_CS Pin: BCM 8
// Channel 1 SPI_CS Pin: BCM 7
// Interupt Pin: BCM25
const int SPI_CS_PIN  = BCM8;
const int CAN_INT_PIN = BCM25;
#else

// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#endif


#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif

unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned long id = 0;
unsigned char buf[8];
char str[20];

//States
int DriveMode = 1;

//Inputs
int ThrotRamp;
int ThrotMax;
int Gear;

int motorrpm;

//Outputs

int32_t ChangeThrotRamp;
int32_t ChangeThrotMax = 100;
int GearChange = 1;
uint8_t CTR1 = ChangeThrotMax >> 0;
uint8_t CTR2 = ChangeThrotMax >> 8;
uint8_t CTR3 = ChangeThrotMax >> 16;
uint8_t CTR4 = ChangeThrotMax >> 24;


// messages to set
//unsigned char Changemap[8] = {0x23, 0x00, 0x21, 25, CTR1, CTR2, CTR3, CTR4}; // currently just changing throttle Max, so set to one parameter
//set buttons to pins

void setup() {
  Serial.begin(115200);
  while (!Serial) {};
  //attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), MCP2515_ISR, FALLING); // start interrupt
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
    Serial.println("CAN init fail, retry...");
    delay(100);
  }
  Serial.println("CAN init ok!");

  /*
      set mask, set both the mask to 0x3ff
  */
  CAN.init_Mask(0, 0, 0x3ff);                         // there are 2 mask in mcp2515, you need to set both of them
  CAN.init_Mask(1, 0, 0x3ff);


  /*
      set filter, we can mask up to 6 IDs
  */
  CAN.init_Filt(0, 0, 0x394);                          // RPM receive
  CAN.init_Filt(1, 0, 0x395);                          // Trotramp, ThrotMax and RPM

  CAN.init_Filt(2, 0, 0x467);                          // there are 6 filter in mcp2515
  CAN.init_Filt(3, 0, 0x07);                          // there are 6 filter in mcp2515
  CAN.init_Filt(4, 0, 0x08);                          // there are 6 filter in mcp2515
  CAN.init_Filt(5, 0, 0x09);                          // there are 6 filter in mcp2515


  // pin setup
  pinMode(LightSport, OUTPUT);
  pinMode(LightEco, OUTPUT);
  pinMode(LightDrift, OUTPUT);

  pinMode(ButtonSport, INPUT_PULLUP);
  pinMode(ButtonEco, INPUT_PULLUP);
  pinMode(ButtonDrift, INPUT_PULLUP);
}
void MCP2515_ISR() {
  flagRecv = 1;
}

void canbusread() {
  if (flagRecv) {                // check if get data

    flagRecv = 0;                // clear flag
    unsigned int id = CAN.getCanId();
    // Serial.print(id);
    if (id == 0x394) {

      motorrpm = (buf[0] + buf[1]); //read motorrpm
      Serial.print(motorrpm);
    }
    if (id == 0x395) { //Trotramp, ThrotMax and RPM and gear
      CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
      Gear = (buf[7]);
      // Serial.print(buf[7]);
      //Serial.print(" ");
      ThrotRamp = (buf[2]);
      ThrotMax = (buf[4]);
    }

  }
}



void setstates() {
  if (Gear == 1 and ThrotMax == 100) { //SportMode
    DriveMode = 1;
    Serial.println("Sportmode");

    //Also light LED sport
  }
  if (ThrotMax == 10 and Gear == 1) { //EcoMode
    DriveMode = 2;
    Serial.println("Eco Mode");
    //Also light LED Eco
  }
  if (Gear == 0 and ThrotMax == 100) { //Drift
    DriveMode = 3;
    Serial.println("Drift Mode");
    //Also light LED Drift
  }


}

void ButtonPress() {
  byte buttonstate1 = digitalRead(ButtonSport);
  if (buttonstate1 == LOW) { // Sport
  
    
      ChangeThrotMax = 3200;
      GearChange = 32;
      CTR1 = ChangeThrotMax >> 0;
      CTR2 = ChangeThrotMax >> 8;
      CTR3 = ChangeThrotMax >> 16;
      CTR4 = ChangeThrotMax >> 24;
      unsigned char Changemap[8] = {0x23, 0x00, 0x21, 0x19, CTR1, CTR2, CTR3, CTR4}; // currently just changing throttle Max, so set to one parameter
      //unsigned char Changemap[8] = {0x23, 0x01, 0x20, 25, CTR1, CTR2, CTR3, CTR4}; // currently just changing throttle Max, so set to one parameter
      CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap); //25 is value of throt max in params.h
      CTR1 = GearChange >> 0;
      CTR2 = GearChange >> 8;
      CTR3 = GearChange >> 16;
      CTR4 = GearChange >> 24;
      unsigned char Changemap2[8] = {0x23, 0x00, 0x21, 0x1B, CTR1, CTR2, CTR3, CTR4}; //25 is value of throt max in params.h
      //unsigned char Changemap2[8] = {0x23, 0x01, 0x20, 27, CTR1, CTR2, CTR3, CTR4}; //25 is value of throt max in params.h
      CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap2);
      Serial.println("Changing to Sport mode");
      DriveMode = 1;
    }
  




  byte buttonstate2 = digitalRead(ButtonEco);
  if (buttonstate2 == LOW) { //Eco
    Serial.println("Eco button Pressed");
    ChangeThrotMax = 640;
    GearChange = 32;
    CTR1 = ChangeThrotMax >> 0;
    CTR2 = ChangeThrotMax >> 8;
    CTR3 = ChangeThrotMax >> 16;
    CTR4 = ChangeThrotMax >> 24;
    unsigned char Changemap[8] = {0x23, 0x00, 0x21, 0x19, CTR1, CTR2, CTR3, CTR4}; // currently just changing throttle Max, so set to one parameter
    CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap); //25 is value of throt max in params.h
    CTR1 = GearChange >> 0;
    CTR2 = GearChange >> 8;
    CTR3 = GearChange >> 16;
    CTR4 = GearChange >> 24;
    unsigned char Changemap2[8] = {0x23, 0x00, 0x21, 0x1B, CTR1, CTR2, CTR3, CTR4}; //27 is value of gear in params.h
    CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap2);
    Serial.println("Changing to Eco mode");
    DriveMode = 2;
  }

byte buttonstate3 = digitalRead(ButtonDrift);
if (buttonstate3 == LOW) { //Drift
  Serial.println("Drift button Pressed");
  ChangeThrotMax = 3200;
  GearChange = 0;// still to implement
  CTR1 = ChangeThrotMax >> 0;
  CTR2 = ChangeThrotMax >> 8;
  CTR3 = ChangeThrotMax >> 16;
  CTR4 = ChangeThrotMax >> 24;
  unsigned char Changemap[8] = {0x23, 0x00, 0x21, 0x19, CTR1, CTR2, CTR3, CTR4}; //25 is value of throt max in params.h
  CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap);
  CTR1 = GearChange >> 0;
  CTR2 = GearChange >> 8;
  CTR3 = GearChange >> 16;
  CTR4 = GearChange >> 24;
  unsigned char Changemap2[8] = {0x23, 0x00 , 0x21, 0x1B, CTR1, CTR2, CTR3, CTR4}; //27 is value of gear in params.h
  CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap2);

  Serial.println("Changing to Drift mode");
  DriveMode = 3;
}



}
void LightLED() {
  switch (DriveMode) {
    case 1:
      digitalWrite(LightSport, HIGH);
      digitalWrite(LightEco, LOW);
      digitalWrite(LightDrift, LOW);
      Serial.println("sport light on");
      break;

    case 2:
      digitalWrite(LightEco, HIGH);
      digitalWrite(LightSport, LOW);
      digitalWrite(LightDrift, LOW);
      Serial.println("Eco light on");
      break;

    case 3:
      digitalWrite(LightDrift, HIGH);
      digitalWrite(LightEco, LOW);
      digitalWrite(LightSport, LOW);
      Serial.println("Drift light on");
      break;


  }


}


void loop() {
  canbusread();
  //setstates();
  LightLED();
  ButtonPress();
}
