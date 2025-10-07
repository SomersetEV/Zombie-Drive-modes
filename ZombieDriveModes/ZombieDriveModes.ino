

//Change drive modes on EVcontrols, using Arduino Uno and canshield
// Ben Bament 2024-2025

// based on orignal code from:
// loovee, 2014-6-13
#include <SPI.h>
#include <Chrono.h>
Chrono myChrono;

#define CAN_2515
// #define CAN_2518FD

// Set SPI CS Pin according to your hardware

#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
// For Wio Terminal w/ MCP2518FD RPi Hat：
// Channel 0 SPI_CS Pin: BCM 8
// Channel 1 SPI_CS Pin: BCM 7
// Interupt Pin: BCM25
const int SPI_CS_PIN = BCM8;
const int CAN_INT_PIN = BCM25;
#else

// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#endif


#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN);  // Set CS pin
#endif

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN);  // Set CS pin
#endif

//drive mode
//input buttons
#define ButtonSport 3
#define ButtonRegen 5
#define ButtonChill 7

// output for LEDs
#define LightSport 6
#define LightRegen 7
#define LightChill 8

char str[20];

//States
int DriveMode = 1;
int SportMode = 0;
int ChillMode = 0;
int RegenMode = 0;

//Inputs
int soc;
int32_t dischargevoltagelimit;
int32_t dischargecurrentlimit;
int32_t chargevoltagelimit;
int32_t chargecurrentlimit;
int32_t batteryvoltage;

// parameter IDs
int Motorlimits = 0x696;

//Outputs
int32_t PowerMax;
int32_t RegenMax;
int32_t Regenlimit;
int32_t Dischargelimit;




void setup() {
  SERIAL_PORT_MONITOR.begin(115200);

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {  // init can bus : baudrate = 500k
    SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
    delay(100);
  }
  SERIAL_PORT_MONITOR.println("CAN init ok!");

  // pin setup
  pinMode(LightSport, OUTPUT);
  pinMode(LightRegen, OUTPUT);
  pinMode(LightChill, OUTPUT);

  pinMode(ButtonSport, INPUT_PULLUP);
  pinMode(ButtonRegen, INPUT_PULLUP);
  pinMode(ButtonChill, INPUT_PULLUP);
}

void canbusread() {
  unsigned char len = 0;
  unsigned char buf[8];

  if (CAN_MSGAVAIL == CAN.checkReceive()) {  // check if data coming
    CAN.readMsgBuf(&len, buf);               // read data,  len: data length, buf: data buf

    unsigned long canId = CAN.getCanId();
    if (canId == 0x351) {
      dischargevoltagelimit = ((uint16_t)buf[6] | ((uint16_t)buf[7] << 8)) * 0.1;  // in 0.1 scale
      dischargecurrentlimit = ((uint16_t)buf[4] | ((uint16_t)buf[5] << 8)) * 0.1;  // in 0.1 scale
      chargevoltagelimit = ((uint16_t)buf[0] | ((uint16_t)buf[1] << 8)) * 0.1;     // in 0.1 scale
      chargecurrentlimit = ((uint16_t)buf[2] | ((uint16_t)buf[3] << 8)) * 0.1;     // in 0.1 scale
      //SERIAL_PORT_MONITOR.print("discharge current limit: ");
      //SERIAL_PORT_MONITOR.println(dischargecurrentlimit);
      //SERIAL_PORT_MONITOR.print("charge voltage limit: ");
      //SERIAL_PORT_MONITOR.println(chargevoltagelimit);
    }
    if (canId == 0x356) {
      batteryvoltage = ((uint16_t)buf[0] | ((uint16_t)buf[1] << 8)) * 0.1;  // in 0.1 scale
      //SERIAL_PORT_MONITOR.print("battery voltage:");
      //SERIAL_PORT_MONITOR.print(batteryvoltage);
      //SERIAL_PORT_MONITOR.println("V");
    }
  }
}


void ButtonPress() {
  byte buttonstate1 = digitalRead(ButtonSport);
  if (buttonstate1 == LOW) {  // Sport

    SERIAL_PORT_MONITOR.println("Sport button Pressed");
    SportMode = 1;
  } else {
    SportMode = 0;
  }

  byte buttonstate2 = digitalRead(ButtonRegen);
  if (buttonstate2 == LOW) {  //Regen
    SERIAL_PORT_MONITOR.println("Regen button Pressed");

    
    RegenMode = 1;
  } else {
    RegenMode = 0;
  }
  byte buttonstate3 = digitalRead(ButtonChill);
  if (buttonstate3 == LOW) {  //Chill
    SERIAL_PORT_MONITOR.println("Chill button Pressed");

    
    ChillMode = 1;
  } else {
    ChillMode = 0;
  }
}
void processlimits() {
  Regenlimit = batteryvoltage * chargecurrentlimit;
  Dischargelimit = batteryvoltage * dischargecurrentlimit;
  Dischargelimit = Dischargelimit * 0.001;
  Regenlimit = Regenlimit * 0.001;


  if ((SportMode == 1 & ChillMode == 1) or (SportMode == 0 & ChillMode == 0)) {
    DriveMode = 3;
  } else if (SportMode == 1) {
    DriveMode = 1;
  } else if (ChillMode == 1) {
    DriveMode = 2;
  }

  if (RegenMode == 1) {
    RegenMax = 10;
    SERIAL_PORT_MONITOR.println("Changing to Regen mode");
  } else {
    RegenMax = 2;
  }


  switch (DriveMode) {
    case 1:  //sport mode
      PowerMax = 200;
      SERIAL_PORT_MONITOR.println("Changing to Sport mode");
      break;

    case 2:  // chill mode
      PowerMax = 50;
      SERIAL_PORT_MONITOR.println("Changing to Chill mode");
      break;

    case 3:  //default
      PowerMax = 100;
      break;
  }

  if (PowerMax > Dischargelimit) {  //BMS overrides
    PowerMax = Dischargelimit;
  }
  if (RegenMax > Regenlimit) {  //BMS overrides
    RegenMax = Regenlimit;
  }


  RegenMax = RegenMax * 100;
  PowerMax = PowerMax * 100;

  //  SERIAL_PORT_MONITOR.print("Sending regen limit: ");
  //  SERIAL_PORT_MONITOR.println(RegenMax);
  //  SERIAL_PORT_MONITOR.print("Sending Power limit:");
  //  SERIAL_PORT_MONITOR.println(PowerMax);



  uint8_t CTR1 = PowerMax >> 8;    // MSB of PowerMax first
  uint8_t CTR2 = PowerMax & 0xFF;  // LSB second
  uint8_t CTR3 = RegenMax >> 8;    // MSB of RegenMax first
  uint8_t CTR4 = RegenMax & 0xFF;  // LSB second

  unsigned char Changemap5[8] = { CTR1, CTR2, CTR3, CTR4, 0x00, 0x00, 0x00, 0x00 };
  CAN.MCP_CAN::sendMsgBuf(0x696, 0, 8, Changemap5);  //0x696 is motor limits
}



void loop() {
  canbusread();

  if (myChrono.hasPassed(100)) {  // elapsed(100) returns 1 if 100ms have passed.
    myChrono.restart();           // restart the Chrono
    ButtonPress();
    processlimits();
  }
}
/*********************************************************************************************************
    END FILE
*********************************************************************************************************/
