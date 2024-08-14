// demo: CAN-BUS Shield, receive data with check mode
// send data coming to fast, such as less than 10ms, you can use this way
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
const int SPI_CS_PIN  = BCM8;
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
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin
#endif

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif

//drive mode
//input buttons
#define ButtonSport 5
#define ButtonEco 4
#define ButtonDrift 3

// output for LEDs
#define LightSport 6
#define LightEco 7
#define LightDrift 8

char str[20];

//States
int DriveMode = 1;

//Inputs
int ThrotRamp;
int ThrotMax;
int Gear;
int RegenMax;
int rpm = 50;
int regenendrpm;
int regenstate = 1; //default regen state  on

// parameter IDs
int throtmaxid = 0x19;
int throtrampid = 0xD;
int regenmaxid = 0x3D;
int gearchangeid = 0x1B;
int regenendrpmid = 0x7E;


//Outputs

int32_t ChangeThrotRamp;
int32_t ChangeThrotMax = 100;
int GearChange = 1;
uint8_t CTR1 = ChangeThrotMax >> 0;
uint8_t CTR2 = ChangeThrotMax >> 8;
uint8_t CTR3 = ChangeThrotMax >> 16;
uint8_t CTR4 = ChangeThrotMax >> 24;



void setup() {
  SERIAL_PORT_MONITOR.begin(115200);

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
    SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
    delay(100);
  }
  SERIAL_PORT_MONITOR.println("CAN init ok!");

  // pin setup
  pinMode(LightSport, OUTPUT);
  pinMode(LightEco, OUTPUT);
  pinMode(LightDrift, OUTPUT);

  pinMode(ButtonSport, INPUT_PULLUP);
  pinMode(ButtonEco, INPUT_PULLUP);
  pinMode(ButtonDrift, INPUT_PULLUP);

}

void canbusread() {
  unsigned char len = 0;
  unsigned char buf[8];

  if (CAN_MSGAVAIL == CAN.checkReceive()) {         // check if data coming
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

    unsigned long canId = CAN.getCanId();
    if (canId == 394) {
      rpm = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
      Serial.println(rpm);
     
    }
  }
}


void ButtonPress() {
  byte buttonstate1 = digitalRead(ButtonSport);
  if (buttonstate1 == LOW) { // Sport

    // change params, all values * 32.
    ChangeThrotMax = 3200;
    GearChange = 32;
    RegenMax = -20 * 32;
    ChangeThrotRamp = 1600;


    // throt max
    CTR1 = ChangeThrotMax >> 0;
    CTR2 = ChangeThrotMax >> 8;
    CTR3 = ChangeThrotMax >> 16;
    CTR4 = ChangeThrotMax >> 24;
    unsigned char Changemap[8] = {0x23, 0x00, 0x21, throtmaxid, CTR1, CTR2, CTR3, CTR4}; 
    CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap); //25 is value of throt max in params.h

    // gear change
    CTR1 = GearChange >> 0;
    CTR2 = GearChange >> 8;
    CTR3 = GearChange >> 16;
    CTR4 = GearChange >> 24;
    unsigned char Changemap2[8] = {0x23, 0x00, 0x21, gearchangeid, CTR1, CTR2, CTR3, CTR4}; 
    CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap2);

    //ThrotRamp
    CTR1 = ChangeThrotRamp >> 0;
    CTR2 = ChangeThrotRamp >> 8;
    CTR3 = ChangeThrotRamp >> 16;
    CTR4 = ChangeThrotRamp >> 24;
    unsigned char Changemap3[8] = {0x23, 0x00, 0x21, throtrampid, CTR1, CTR2, CTR3, CTR4}; 
    CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap3); //25 is value of throt max in params.h

    // Regen
    CTR1 = RegenMax >> 0;
    CTR2 = RegenMax >> 8;
    CTR3 = RegenMax >> 16;
    CTR4 = RegenMax >> 24;
    unsigned char Changemap4[8] = {0x23, 0x00, 0x21, regenmaxid, CTR1, CTR2, CTR3, CTR4}; 
    //CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap4); //25 is value of throt max in params.h  // dont set regen anymore, regenadjust function does this.


    Serial.println("Changing to Sport mode");
    DriveMode = 1;
  }





  byte buttonstate2 = digitalRead(ButtonEco);
  if (buttonstate2 == LOW) { //Eco
    Serial.println("Eco button Pressed");
    // change params, values * 32
    ChangeThrotMax = 40 * 32;
    GearChange = 32;
    RegenMax = -20 * 32;
    ChangeThrotRamp = 3200;
    // throtmax
    CTR1 = ChangeThrotMax >> 0;
    CTR2 = ChangeThrotMax >> 8;
    CTR3 = ChangeThrotMax >> 16;
    CTR4 = ChangeThrotMax >> 24;
    unsigned char Changemap[8] = {0x23, 0x00, 0x21, throtmaxid, CTR1, CTR2, CTR3, CTR4}; 
    CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap); //25 is value of throt max in params.h
    // gear change
    CTR1 = GearChange >> 0;
    CTR2 = GearChange >> 8;
    CTR3 = GearChange >> 16;
    CTR4 = GearChange >> 24;
    unsigned char Changemap2[8] = {0x23, 0x00, 0x21, gearchangeid, CTR1, CTR2, CTR3, CTR4}; 
    CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap2);
    //ThrotRamp
    CTR1 = ChangeThrotRamp >> 0;
    CTR2 = ChangeThrotRamp >> 8;
    CTR3 = ChangeThrotRamp >> 16;
    CTR4 = ChangeThrotRamp >> 24;
    unsigned char Changemap3[8] = {0x23, 0x00, 0x21, throtrampid, CTR1, CTR2, CTR3, CTR4}; 
    CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap3); //25 is value of throt max in params.h

    // Regen
    CTR1 = RegenMax >> 0;
    CTR2 = RegenMax >> 8;
    CTR3 = RegenMax >> 16;
    CTR4 = RegenMax >> 24;
    unsigned char Changemap4[8] = {0x23, 0x00, 0x21, regenmaxid, CTR1, CTR2, CTR3, CTR4}; 
    //CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap4); //25 is value of throt max in params.h  // dont set regen anymore, regenadjust function does this.

    Serial.println("Changing to Eco mode");
    DriveMode = 2;
  }

  byte buttonstate3 = digitalRead(ButtonDrift);
  if (buttonstate3 == LOW) { //Drift
    Serial.println("Drift button Pressed");
    // change params, values * 32
    ChangeThrotMax = 3200;
    GearChange = 0;
    RegenMax = 0;
    ChangeThrotRamp = 3200;
    //throt max
    CTR1 = ChangeThrotMax >> 0;
    CTR2 = ChangeThrotMax >> 8;
    CTR3 = ChangeThrotMax >> 16;
    CTR4 = ChangeThrotMax >> 24;
    unsigned char Changemap[8] = {0x23, 0x00, 0x21, throtmaxid, CTR1, CTR2, CTR3, CTR4}; 
    CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap);

    // gear change
    CTR1 = GearChange >> 0;
    CTR2 = GearChange >> 8;
    CTR3 = GearChange >> 16;
    CTR4 = GearChange >> 24;
    unsigned char Changemap2[8] = {0x23, 0x00 , 0x21, gearchangeid, CTR1, CTR2, CTR3, CTR4}; 
    CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap2);

    //ThrotRamp
    CTR1 = ChangeThrotRamp >> 0;
    CTR2 = ChangeThrotRamp >> 8;
    CTR3 = ChangeThrotRamp >> 16;
    CTR4 = ChangeThrotRamp >> 24;
    unsigned char Changemap3[8] = {0x23, 0x00, 0x21, throtrampid, CTR1, CTR2, CTR3, CTR4}; 
     CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap3); 

    // Regen
    CTR1 = RegenMax >> 0;
    CTR2 = RegenMax >> 8;
    CTR3 = RegenMax >> 16;
    CTR4 = RegenMax >> 24;
    unsigned char Changemap4[8] = {0x23, 0x00, 0x21, regenmaxid, CTR1, CTR2, CTR3, CTR4};
     CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap4); 


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
      // Serial.println("sport light on");
      break;

    case 2:
      digitalWrite(LightEco, HIGH);
      digitalWrite(LightSport, LOW);
      digitalWrite(LightDrift, LOW);
      //  Serial.println("Eco light on");
      break;

    case 3:
      digitalWrite(LightDrift, HIGH);
      digitalWrite(LightEco, LOW);
      digitalWrite(LightSport, LOW);
      //  Serial.println("Drift light on");
      break;


  }


}

void Regenendadjust() { // adjust regenmax to avoid regen being active around regenendrpm

  if (rpm <  500 && regenstate == 1) {
    regenendrpm = 0 * 32; //actually setting regen max
    CTR1 =  regenendrpm >> 0;
    CTR2 =  regenendrpm >> 8;
    CTR3 =  regenendrpm >> 16;
    CTR4 =  regenendrpm >> 24;
    unsigned char Changemap5[8] = {0x23, 0x00, 0x21, regenmaxid, CTR1, CTR2, CTR3, CTR4};
    CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap5); //25 is value of throt max in params.h
    regenstate = 0;
  }

  if (rpm > 1000 && regenstate == 0 && DriveMode < 3) { // adjust regenmax to avoid regen being active around regenendrpm, dont enable regen in drift mode, 
    regenendrpm = -25 * 32; //actually setting regen max
    CTR1 =  regenendrpm >> 0;
    CTR2 =  regenendrpm >> 8;
    CTR3 =  regenendrpm >> 16;
    CTR4 =  regenendrpm >> 24;
    unsigned char Changemap6[8] = {0x23, 0x00, 0x21, regenmaxid, CTR1, CTR2, CTR3, CTR4}; 
    CAN.MCP_CAN::sendMsgBuf(0x603, 0, 8, Changemap6);
    regenstate = 1;
  }

}


void loop() {
  canbusread();
  //setstates();
  if (myChrono.hasPassed(50) ) { // elapsed(50) returns 1 if 50ms have passed.
    myChrono.restart();  // restart the Chrono
    ButtonPress();
    Regenendadjust();
    LightLED();
  }

}
/*********************************************************************************************************
    END FILE
*********************************************************************************************************/
