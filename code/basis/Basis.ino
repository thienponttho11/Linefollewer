/* ============================================================
   Basis - linefollower
   ============================================================ */

#include "SerialCommand.h"
#include "EEPROMAnything.h"
#include <SoftwareSerial.h>

#define SerialPort Serial
#define Baudrate 115200

// Bluetooth (HC-05)
SoftwareSerial BT(2, 3); // RX, TX

SerialCommand sCmdUSB(SerialPort);
SerialCommand sCmdBT(BT);
SerialCommand* activeCmd = nullptr;

// ---------------- parameters ----------------
struct param_t {
  unsigned long cycleTime; // microseconden
  bool debug;
} params;

bool running = false;

// timing
unsigned long previous = 0;

// voorbeeld output
#define LED_PIN 13
bool ledState = false;

// ---------------- defaults ----------------
void setDefaults() {
  params.cycleTime = 500000UL; // 500 ms
  params.debug = false;
}

// ---------------- setup ----------------
void setup() {
  pinMode(LED_PIN, OUTPUT);

  SerialPort.begin(Baudrate);
  BT.begin(9600);

  // commands USB
  sCmdUSB.addCommand("set", onSet);
  sCmdUSB.addCommand("start", onStart);
  sCmdUSB.addCommand("stop", onStop);
  sCmdUSB.addCommand("status", onStatus);
  sCmdUSB.setDefaultHandler(onUnknownCommand);

  // commands BT
  sCmdBT.addCommand("set", onSet);
  sCmdBT.addCommand("start", onStart);
  sCmdBT.addCommand("stop", onStop);
  sCmdBT.addCommand("status", onStatus);
  sCmdBT.setDefaultHandler(onUnknownCommand);

  // EEPROM
  EEPROM_readAnything(0, params);
  if (params.cycleTime == 0) {
    setDefaults();
    EEPROM_writeAnything(0, params);
  }

  SerialPort.println(F("Template ready"));
  BT.println(F("Template ready"));
}

// ---------------- loop ----------------
void loop() {
  if (SerialPort.available()) {
    activeCmd = &sCmdUSB;
    sCmdUSB.readSerial();
  }

  if (BT.available()) {
    activeCmd = &sCmdBT;
    sCmdBT.readSerial();
  }

  unsigned long now = micros();
  if (running && (now - previous >= params.cycleTime)) {
    previous = now;
    stepCycle();
  }
}

// ---------------- cyclische code ----------------
void stepCycle() {
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState);

  if (params.debug) {
    SerialPort.println(F("cycle"));
    BT.println(F("cycle"));
  }
}

// ---------------- command handlers ----------------
void onSet() {
  char* p = activeCmd->next();
  char* v = activeCmd->next();

  if (!p || !v) return;

  if (strcmp(p, "cycle") == 0) params.cycleTime = atol(v);
  else if (strcmp(p, "debug") == 0) params.debug = (strcmp(v,"on")==0);

  EEPROM_writeAnything(0, params);

  SerialPort.println(F("OK"));
  BT.println(F("OK"));
}

void onStart() {
  running = true;
  SerialPort.println(F("started"));
  BT.println(F("started"));
}

void onStop() {
  running = false;
  digitalWrite(LED_PIN, LOW);
  SerialPort.println(F("stopped"));
  BT.println(F("stopped"));
}

void onStatus() {
  SerialPort.print(F("cycleTime: "));
  SerialPort.println(params.cycleTime);
  SerialPort.print(F("running: "));
  SerialPort.println(running ? "yes":"no");

  BT.print(F("cycleTime: "));
  BT.println(params.cycleTime);
  BT.print(F("running: "));
  BT.println(running ? "yes":"no");
}

void onUnknownCommand(char* cmd) {
  SerialPort.print(F("unknown: "));
  SerialPort.println(cmd);
  BT.print(F("unknown: "));
  BT.println(cmd);
}
