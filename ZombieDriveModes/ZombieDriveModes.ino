

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
// Uno pin map -- keep this list in sync when changing any pin below.
//   0, 1  : Serial (do not use)
//   2     : CAN INT (not currently used, polling instead)
//   9     : CAN CS
//   10-13 : SPI / CAN shield
// Free: 3, 4, 5, 6, 7, 8, A0-A5

//input buttons
#define ButtonSport 3
#define ButtonRegen 5
#define ButtonChill 4  // was 7, which collided with LightRegen

// output for LEDs
#define LightSport 6
// WARNING: pin 7 is reserved for LightRegen but is NOT driven -- the
// digitalWrite calls were removed after 83ede9e. If they are ever restored,
// pin 7 must not be driven while anything else reads it. That collision
// (ButtonChill was also on 7) is what jammed ChillMode.
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

// BMS freshness tracking. Both start "never received" so the limp-home
// fallback applies at boot as well as on a mid-drive dropout.
#define BMS_TIMEOUT_MS 1000
#define LIMP_DISCHARGE_KW 20
#define LIMP_REGEN_KW 4
unsigned long last351ms = 0;
unsigned long last356ms = 0;
bool seen351 = false;
bool seen356 = false;

// parameter IDs
const unsigned long Motorlimits = 0x696;

//Outputs
int32_t PowerMax;
int32_t RegenMax;
int32_t Regenlimit;
int32_t Dischargelimit;

// previous values, so serial prints only fire on change
int prevDriveMode = -1;
int prevRegenMode = -1;
bool prevLimp = false;
byte prevSendResult = CAN_OK;
unsigned long lastSendErrms = 0;




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

  // The MCP2515 only has 2 RX buffers and readMsgBuf drains one of them per
  // call, so a single read per loop() overruns on a busy bus and drops frames.
  // Drain what is queued, but cap the iterations so a saturated bus cannot
  // starve the 100ms task below.
  byte guard = 0;
  while (CAN_MSGAVAIL == CAN.checkReceive() && guard < 8) {
    guard++;

    // CAN_NOMSG here just means the flag cleared between checkReceive and the
    // read -- nothing left to drain, so stop.
    if (CAN_OK != CAN.readMsgBuf(&len, buf)) {  // read data, len: data length, buf: data buf
      break;
    }

    unsigned long canId = CAN.getCanId();
    // Length is checked per-ID against the bytes that ID actually uses. A
    // blanket len<8 test would discard a valid short 0x356 and leave the
    // limp-home fallback latched on forever.
    if (canId == 0x351 && len >= 8) {
      dischargevoltagelimit = ((uint16_t)buf[6] | ((uint16_t)buf[7] << 8)) * 0.1;  // in 0.1 scale
      dischargecurrentlimit = ((uint16_t)buf[4] | ((uint16_t)buf[5] << 8)) * 0.1;  // in 0.1 scale
      chargevoltagelimit = ((uint16_t)buf[0] | ((uint16_t)buf[1] << 8)) * 0.1;     // in 0.1 scale
      chargecurrentlimit = ((uint16_t)buf[2] | ((uint16_t)buf[3] << 8)) * 0.1;     // in 0.1 scale
      last351ms = millis();
      seen351 = true;
    }
    if (canId == 0x356 && len >= 2) {
      batteryvoltage = ((uint16_t)buf[0] | ((uint16_t)buf[1] << 8)) * 0.1;  // in 0.1 scale
      last356ms = millis();
      seen356 = true;
    }
  }
}


void ButtonPress() {
  // These are latching switches read every 100ms, so printing on "pressed"
  // printed on every cycle the button was held. Serial.print blocks once the
  // 64-byte TX buffer fills, and blocked time is time the CAN RX buffers
  // overflow. State changes are reported in processlimits() instead.
  SportMode = (digitalRead(ButtonSport) == LOW) ? 1 : 0;
  RegenMode = (digitalRead(ButtonRegen) == LOW) ? 1 : 0;
  ChillMode = (digitalRead(ButtonChill) == LOW) ? 1 : 0;
}
void processlimits() {
  unsigned long now = millis();

  // BMS data is stale if either frame has never arrived or has stopped. Both
  // cases take the same branch, so the limp-home limit also applies at boot
  // instead of silently clamping PowerMax to 0 before CAN comes up.
  bool stale = (!seen351 || !seen356 || (now - last351ms) > BMS_TIMEOUT_MS || (now - last356ms) > BMS_TIMEOUT_MS);

  if (stale) {
    Dischargelimit = LIMP_DISCHARGE_KW;
    Regenlimit = LIMP_REGEN_KW;
  } else {
    Regenlimit = batteryvoltage * chargecurrentlimit;
    Dischargelimit = batteryvoltage * dischargecurrentlimit;
    Dischargelimit = Dischargelimit * 0.001;
    Regenlimit = Regenlimit * 0.001;
  }

  if (stale != prevLimp) {
    SERIAL_PORT_MONITOR.println(stale ? "WARNING: BMS data stale - limp home limits"
                                      : "BMS data OK");
    prevLimp = stale;
  }

  if ((SportMode == 1 && ChillMode == 1) || (SportMode == 0 && ChillMode == 0)) {
    DriveMode = 3;
  } else if (SportMode == 1) {
    DriveMode = 1;
  } else if (ChillMode == 1) {
    DriveMode = 2;
  }

  if (RegenMode == 1) {
    RegenMax = 20;
  } else {
    RegenMax = 4;
  }
  if (RegenMode != prevRegenMode) {
    SERIAL_PORT_MONITOR.println(RegenMode == 1 ? "Changing to Regen mode" : "Regen mode off");
    prevRegenMode = RegenMode;
  }

  switch (DriveMode) {
    case 1:  //sport mode
      PowerMax = 300;
      break;

    case 2:  // chill mode
      PowerMax = 100;
      break;

    case 3:  //default
      PowerMax = 150;
      break;
  }

  if (DriveMode != prevDriveMode) {
    switch (DriveMode) {
      case 1: SERIAL_PORT_MONITOR.println("Changing to Sport mode"); break;
      case 2: SERIAL_PORT_MONITOR.println("Changing to Chill mode"); break;
      case 3: SERIAL_PORT_MONITOR.println("Changing to Default mode"); break;
    }
    prevDriveMode = DriveMode;
  }

  if (PowerMax > Dischargelimit) {  //BMS overrides
    PowerMax = Dischargelimit;
  }
  if (RegenMax > Regenlimit) {  //BMS overrides
    RegenMax = Regenlimit;
  }


  RegenMax = RegenMax * 100;
  PowerMax = PowerMax * 100;

  // Final encoding guard before the >>8 / &0xFF truncation below. In normal
  // operation PowerMax/RegenMax are already bounded by the mode switch (max
  // 200 -> 20000) and the BMS override only ever lowers them, so this should
  // never fire -- it exists so a future mode value or scaling change cannot
  // silently wrap and transmit a LARGER limit than intended.
  if (PowerMax < 0) PowerMax = 0;
  if (PowerMax > 65535) PowerMax = 65535;
  if (RegenMax < 0) RegenMax = 0;
  if (RegenMax > 65535) RegenMax = 65535;

  uint8_t CTR1 = PowerMax >> 8;    // MSB of PowerMax first
  uint8_t CTR2 = PowerMax & 0xFF;  // LSB second
  uint8_t CTR3 = RegenMax >> 8;    // MSB of RegenMax first
  uint8_t CTR4 = RegenMax & 0xFF;  // LSB second

  unsigned char Changemap5[8] = { CTR1, CTR2, CTR3, CTR4, 0x00, 0x00, 0x00, 0x00 };
  // sendMsgBuf busy-waits up to ~500us (TIMEOUTVALUE 50 x 10us) for a free TX
  // buffer and again for the send to complete, so a bus with nothing ACKing
  // costs ~1ms per cycle and previously failed silently.
  byte sendResult = CAN.sendMsgBuf(Motorlimits, 0, 8, Changemap5);  //0x696 is motor limits
  if (sendResult != CAN_OK) {
    // Rate limit: report on the first failure, then at most once a second, so
    // a persistently failing bus cannot flood (and block on) the serial port.
    if (prevSendResult == CAN_OK || (now - lastSendErrms) > 1000) {
      SERIAL_PORT_MONITOR.print("CAN send failed, code ");
      SERIAL_PORT_MONITOR.println(sendResult);
      lastSendErrms = now;
    }
  } else if (prevSendResult != CAN_OK) {
    SERIAL_PORT_MONITOR.println("CAN send recovered");
  }
  prevSendResult = sendResult;
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
