

//Change drive modes on zombieverter, using Arduino Uno and canshield
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
#define ButtonSport 5
#define ButtonRegen 4
#define ButtonChill 3

// output for LEDs
#define LightSport 6
#define LightRegen 7
#define LightChill 8

char str[20];

//States
int DriveMode = 1;

//Inputs
int soc;
int dischargevoltagelimit;
int dischargecurrentlimit;
int chargevoltagelimit;
int chargecurrentlimit;
int batteryvoltage;

// parameter IDs
int Motorlimits = 0x696;

//Outputs
int PowerMax;
int RegenMax;
int Regenlimit;
int Dischargelimit;




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
    if (canId == 351) {
      dischargevoltagelimit = ((uint16_t)buf[6] | ((uint16_t)buf[7] << 8)) * 10;  // in 0.1 scale
      dischargecurrentlimit = ((uint16_t)buf[4] | ((uint16_t)buf[5] << 8)) * 10;  // in 0.1 scale
      chargevoltagelimit = ((uint16_t)buf[0] | ((uint16_t)buf[1] << 8)) * 10;     // in 0.1 scale
      chargecurrentlimit = ((uint16_t)buf[2] | ((uint16_t)buf[3] << 8)) * 10;     // in 0.1 scale
      Serial.println("charge current limit");
      Serial.print(chargecurrentlimit);
    }
    if (canId == 356) {
      batteryvoltage = ((uint16_t)buf[0] | ((uint16_t)buf[1] << 8)) * 10;  // in 0.1 scale
      Serial.println("battery voltage:");
      Serial.print(batteryvoltage);
    }
  }
}


void ButtonPress() {
  byte buttonstate1 = digitalRead(ButtonSport);
  if (buttonstate1 == LOW) {  // Sport

    // change params. units in kW
    PowerMax = 75;
    RegenMax = 26;

    Serial.println("Changing to Sport mode");
    DriveMode = 1;
  }

  byte buttonstate2 = digitalRead(ButtonRegen);
  if (buttonstate2 == LOW) {  //Eco
    Serial.println("Regen button Pressed");

    // change params. units in kW
    PowerMax = 75;
    RegenMax = 26;

    Serial.println("Changing to Regen mode");
    DriveMode = 2;
  }

  byte buttonstate3 = digitalRead(ButtonChill);
  if (buttonstate3 == LOW) {  //Drift
    Serial.println("Chill button Pressed");

    // change params. units in kW
    PowerMax = 75;
    RegenMax = 26;

    Serial.println("Changing to Chill mode");
    DriveMode = 3;
  }
}
void processlimits() {
  Regenlimit = batteryvoltage * chargecurrentlimit;
  Dischargelimit = batteryvoltage * dischargecurrentlimit;
  switch (DriveMode) {
    case 1:  //sport mode
      PowerMax = 200;
      RegenMax = 50;
      if (PowerMax > Dischargelimit) {  //BMS overrides
        PowerMax = Dischargelimit;
      }
      if (RegenMax > Regenlimit) {  //BMS overrides
        RegenMax = Regenlimit;
      }
      break;

    case 2:  // regen mode
      PowerMax = 50;
      RegenMax = 15;
      if (PowerMax > Dischargelimit) {  //BMS overrides
        PowerMax = Dischargelimit;
      }
      if (RegenMax > Regenlimit) {  //BMS overrides
        RegenMax = Regenlimit;
      }
      break;

    case 3:  // chill mode
      PowerMax = 50;
      RegenMax = 10;
      if (PowerMax > Dischargelimit) {  //BMS overrides
        PowerMax = Dischargelimit;
      }
      if (RegenMax > Regenlimit) {  //BMS overrides
        RegenMax = Regenlimit;
      }
      break;
  }

  uint8_t CTR1 = RegenMax >> 0;
  uint8_t CTR2 = RegenMax >> 8;
  uint8_t CTR3 = PowerMax >> 0;
  uint8_t CTR4 = PowerMax >> 8;

  unsigned char Changemap5[8] = { CTR1, CTR2, CTR3, CTR4, 0x00, 0x00, 0x00, 0x00 };
  CAN.MCP_CAN::sendMsgBuf(Motorlimits, 0, 8, Changemap5);
}
void LightLED() {
  switch (DriveMode) {
    case 1:
      digitalWrite(LightSport, HIGH);
      digitalWrite(LightRegen, LOW);
      digitalWrite(LightChill, LOW);
      // Serial.println("sport light on");
      break;

    case 2:
      digitalWrite(LightRegen, HIGH);
      digitalWrite(LightSport, LOW);
      digitalWrite(LightChill, LOW);
      //  Serial.println("Eco light on");
      break;

    case 3:
      digitalWrite(LightChill, HIGH);
      digitalWrite(LightRegen, LOW);
      digitalWrite(LightSport, LOW);
      //  Serial.println("Drift light on");
      break;
  }
}


void loop() {
  canbusread();

  if (myChrono.hasPassed(100)) {  // elapsed(100) returns 1 if 100ms have passed.
    myChrono.restart();           // restart the Chrono
    ButtonPress();
    processlimits();
    LightLED();
  }
}
/*********************************************************************************************************
    END FILE
*********************************************************************************************************/
