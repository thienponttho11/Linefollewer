/* ============================================================
   Line-follower Syntheseproject
   ============================================================ */

#include "SerialCommand.h"
#include "EEPROMAnything.h"
#include <SoftwareSerial.h>


#define SerialPort Serial
#define Baudrate 115200

// Hardware / sensors
#define NUM_SENSORS 8
const uint8_t sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};

// TB6612FNG mapping (volgens jouw opgave)
#define PWMA 5  // LEFT motor PWM
#define AIN1 7
#define AIN2 8

#define PWMB 6  // RIGHT motor PWM
#define BIN1 9
#define BIN2 10

#define STBY 4  // Standby pin (HIGH = active)

bool running = true;  // robot start direct


// SerialCommand object
SoftwareSerial BT(2, 3); // RX, TX HC-05
SerialCommand sCmdUSB(SerialPort); // voor USB monitor
SerialCommand sCmdBT(BT);

SerialCommand* activeCmd = nullptr;


// Timing
unsigned long previous = 0;
unsigned long calculationTime = 0; // max berekeningstijd gemeten (micros)

// Sensor arrays (in RAM)
int sensorValues[NUM_SENSORS];

// Parameterstruct (wordt naar EEPROM geschreven)
struct param_t {
  unsigned long cycleTime;        // micros tussen cycli (bv 10000 = 10 ms)
  int baseSpeed;                  // basis PWM (0..255)
  int maxSpeed;                   // maximum PWM limit
  float Kp;
  float Ki;
  float Kd;
  bool invertLeft;                // invert motor A
  bool invertRight;               // invert motor B
  bool sensorInvert;              // als je sensoren neg/pos wil inverteren
  int sensorMin[NUM_SENSORS];     // calibratie waarden
  int sensorMax[NUM_SENSORS];
} params;

// Defaults (ingevuld als EEPROM leeg / eerste keer)
void setDefaults() {
  params.cycleTime = 10000UL; // 10 ms default
  params.baseSpeed = 90;
  params.maxSpeed = 175;
  params.Kp = 1.20f;
  params.Ki = 0.0f;
  params.Kd = 0.40f;
  params.invertLeft = false;
  params.invertRight = false;
  params.sensorInvert = false;
  // init sensor min/max to extremes (zodat calibratie werkt)
  for (int i=0;i<NUM_SENSORS;i++){
    params.sensorMin[i] = 1023;
    params.sensorMax[i] = 0;
  }
}

// PID runtime state
long lastError = 0;
float integral = 0;
unsigned long lastTime = 0;


// ---------------- setup ----------------
void setup() 
{
  // Serial + commands
  SerialPort.begin(Baudrate);
  BT.begin(9600);           // HC-05
  // voeg commando's toe
  sCmdUSB.addCommand("set", onSet);
  sCmdUSB.addCommand("calibrate", onCalibrate);
  sCmdUSB.addCommand("debug", onDebug);
  sCmdUSB.addCommand("status", onStatus);
  sCmdUSB.addCommand("motortest", onMotorTest);
  sCmdUSB.addCommand("sensortest", onSensorTest);
  sCmdUSB.addCommand("help", onHelp);
  sCmdUSB.addCommand("start", onStart);
  sCmdUSB.addCommand("stop", onStop);
  sCmdUSB.setDefaultHandler(onUnknownCommand);

  sCmdBT.addCommand("set", onSet);
  sCmdBT.addCommand("calibrate", onCalibrate);
  sCmdBT.addCommand("debug", onDebug);
  sCmdBT.addCommand("status", onStatus);
  sCmdBT.addCommand("motortest", onMotorTest);
  sCmdBT.addCommand("sensortest", onSensorTest);
  sCmdBT.addCommand("help", onHelp);
  sCmdBT.addCommand("start", onStart);
  sCmdBT.addCommand("stop", onStop);
  sCmdBT.setDefaultHandler(onUnknownCommand);


  // laadt params uit EEPROM of zet defaults als EEPROM corrupt / leeg
  // probeer lezen - geen foutafhandeling in EEPROMAnything, dus we controleren sensorMin waarden na lezen
  // load -> l aad params uit EEPROM (en overschrijf runtime)
  EEPROM_readAnything(0, params);
  SerialPort.println(F("Parameters loaded from EEPROM."));
  BT.println(F("Parameters loaded from EEPROM."));

  bool needDefaults = false;
  // heuristiek: als cycleTime 0 of sensorMin allemaal 1023 (nog nooit gekalibreerd), dan default zetten
  if (params.cycleTime == 0 || params.baseSpeed < 0 || params.baseSpeed > 255) needDefaults = true;
  // check sensorMax/min plausibiliteit
  int validCount = 0;
  for (int i=0;i<NUM_SENSORS;i++){
    if (params.sensorMax[i] > params.sensorMin[i]) validCount++;
  }
  if (validCount < 1) needDefaults = true;

  if (needDefaults) {
    setDefaults();
    EEPROM_writeAnything(0, params);
  }

  // pin modes motor driver
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // activeer driver

  SerialPort.println(F("ready"));
  BT.println(F("ready"));
  onHelp(); // toon beschikbare commando's bij opstart
  lastTime = micros();

}

// ---------------- main loop ----------------
void loop() 
{
  // verwerk inkomende serial commando's (niet in de cyclische code)
  if (SerialPort.available()) 
  {
    activeCmd = &sCmdUSB;
    sCmdUSB.readSerial();
  }

  if (BT.available()) 
  {
    activeCmd = &sCmdBT;
    sCmdBT.readSerial();
  }

  unsigned long current = micros();

  if (current - previous >= params.cycleTime) 
  {
    previous = current;
    unsigned long start = micros();

    if (running) 
    {
      // cyclische code - houd kort en zonder Serial prints
      stepCycle();
    }
    else
    {
      
    }

    unsigned long dur = micros() - start;
    if (dur > calculationTime) calculationTime = dur; // onthoud max duratie
  }    
}

// ---------------- cyclische stap ----------------
void stepCycle() {
  // 1) lees en normaliseer sensoren (0..1000)
  readCalibratedSensors();

  // 2) bereken positie (gewogen)
  long position = getWeightedPosition(); // 0..(NUM_SENSORS-1)*1000
  long middle = (NUM_SENSORS - 1) * 1000 / 2;
  long error = position - middle;

  // 3) PID berekening
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000.0f; // seconden
  if (dt <= 0) dt = 0.001f;

  integral += error * dt;
  float derivative = (error - lastError) / dt;
  float correction = params.Kp * error + params.Ki * integral + params.Kd * derivative;

  lastError = error;
  lastTime = now;

  // Schaal correction naar bruikbare PWM-delta
  // De deling hieronder is een schaalfactor. Je kunt deze aanpassen tijdens tuning.
  int corr = (int)(correction / 10.0f);

  int leftSpeed  = params.baseSpeed - corr;
  int rightSpeed = params.baseSpeed + corr;

  // beperk en pas invert toe
  leftSpeed = constrain(leftSpeed, -params.maxSpeed, params.maxSpeed);
  rightSpeed = constrain(rightSpeed, -params.maxSpeed, params.maxSpeed);

  if (params.invertLeft) leftSpeed = -leftSpeed;
  if (params.invertRight) rightSpeed = -rightSpeed;

  // 4) stuur motoren aan
  setMotorA(leftSpeed); // links
  setMotorB(rightSpeed); // rechts
}

// ---------------- sensor functies ----------------
void readCalibratedSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(sensorPins[i]);
    int minv = params.sensorMin[i];
    int maxv = params.sensorMax[i];

    int val;
    if (maxv > minv) {
      // map naar 0..1000
      val = map(raw, minv, maxv, 0, 1000);
      val = constrain(val, 0, 1000);
    } else {
      // niet gekalibreerd: gebruik ruwe waarde geschaald
      val = map(raw, 0, 1023, 0, 1000);
      val = constrain(val, 0, 1000);
    }

    if (params.sensorInvert) val = 1000 - val;
    sensorValues[i] = val;
  }
}

long getWeightedPosition() 
{
  long numerator = 0;
  long denominator = 0;
  for (int i = 0; i < NUM_SENSORS; i++) 
  {
    int v = sensorValues[i];
    numerator += (long)v * (long)(i * 1000);
    denominator += v;
  }
  if (denominator == 0) 
  {
    // lijn niet gevonden: fallback naar kant gebaseerd op lastError
    if (lastError >= 0) return (NUM_SENSORS - 1) * 1000;
    else return 0;
  }
  return numerator / denominator;
}

// ---------------- motor functies ----------------
// snelheid: -255 .. +255
void setMotorA(int speed) 
{ // motor A = LINKS
  if (speed > 0) 
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, constrain(speed, 0, 255));
  } 
  else if (speed < 0) 
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, constrain(-speed, 0, 255));
  } 
  else 
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}

void setMotorB(int speed) 
{ // motor B = RECHTS
  if (speed > 0) 
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, constrain(speed, 0, 255));
  } 
  else if (speed < 0) 
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, constrain(-speed, 0, 255));
  } 
  else 
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}

// ---------------- SerialCommand handlers ----------------

// set <param> <value>
// ondersteunde params: kp, ki, kd, base, max, cycle, invertLeft, invertRight, sensorInvert, debug
void onSet() {
  if (!activeCmd) return;

  char* p = activeCmd->next();
  char* v = activeCmd->next();

  if (!p || !v) {
    SerialPort.println(F("set <param> <value>"));
    BT.println(F("set <param> <value>"));
    return;
  }

  if (strcmp(p, "kp") == 0) params.Kp = atof(v);
  else if (strcmp(p, "ki") == 0) params.Ki = atof(v);
  else if (strcmp(p, "kd") == 0) params.Kd = atof(v);
  else if (strcmp(p, "base") == 0) params.baseSpeed = atoi(v);
  else if (strcmp(p, "max") == 0) params.maxSpeed = atoi(v);
  else if (strcmp(p, "cycle") == 0) params.cycleTime = atol(v);
  else if (strcmp(p, "invertLeft") == 0) params.invertLeft = (strcmp(v,"on")==0);
  else if (strcmp(p, "invertRight") == 0) params.invertRight = (strcmp(v,"on")==0);
  else if (strcmp(p, "sensorInvert") == 0) params.sensorInvert = (strcmp(v,"on")==0);
  else if (strcmp(p, "debug") == 0) {
    // debug toggle: éénmalig printen gebeurt in onDebug; debug mode hier schakelt extra prints voor 'status'
    bool dbg = (strcmp(v,"on")==0);
    if (dbg) SerialPort.println(F("debug on (use 'debug' command to show cycle stats)"));
    else SerialPort.println(F("debug off"));
    // we gebruiken debug alleen als flag in status/other prints - niet in cycle
  } else {
    SerialPort.print(F("Unknown set parameter: "));
    SerialPort.println(p);
    return;
  }

  // bewaar parameters meteen naar EEPROM
  EEPROM_writeAnything(0, params);
  SerialPort.print(F("OK set "));
  SerialPort.print(p);
  SerialPort.print(F(" = "));
  SerialPort.println(v);
}

// calibrate [ms]
// Kalibreer sensorMin/max door samples te nemen over opgegeven tijd (ms), default 2000ms
void onCalibrate() {
  char* arg = activeCmd->next();
  unsigned long ms = 2000;
  if (arg) ms = atol(arg);

  SerialPort.print(F("Starting calibration for "));
  SerialPort.print(ms);
  SerialPort.println(F(" ms - move robot over line / surface"));
  BT.print(F("Starting calibration for "));
  BT.print(ms);
  BT.println(F(" ms - move robot over line / surface"));

  unsigned long start = millis();
  // init
  for (int i=0;i<NUM_SENSORS;i++) {
    params.sensorMin[i] = 1023;
    params.sensorMax[i] = 0;
  }

  while (millis() - start < ms) {
    for (int i=0;i<NUM_SENSORS;i++){
      int val = analogRead(sensorPins[i]);
      if (val < params.sensorMin[i]) params.sensorMin[i] = val;
      if (val > params.sensorMax[i]) params.sensorMax[i] = val;
    }
    delay(10); // korte pauze
  }

  // sanity: als waarden ongeldig zijn, zet defaults
  int valid = 0;
  for (int i=0;i<NUM_SENSORS;i++) if (params.sensorMax[i] > params.sensorMin[i]) valid++;
  if (valid == 0) {
    SerialPort.println(F("Calibration failed (no variation). Check wiring/Vcc/GND."));
  } else {
    EEPROM_writeAnything(0, params); // bewaar calibratie
    SerialPort.println(F("Calibration finished. Values:"));
    for (int i=0;i<NUM_SENSORS;i++) {
      SerialPort.print(F("S")); SerialPort.print(i);
      SerialPort.print(F(" min=")); SerialPort.print(params.sensorMin[i]);
      SerialPort.print(F(" max=")); SerialPort.println(params.sensorMax[i]);

      BT.print(F("S")); SerialPort.print(i);
      BT.print(F(" min=")); SerialPort.print(params.sensorMin[i]);
      BT.print(F(" max=")); SerialPort.println(params.sensorMax[i]);
    }
  }
}

// debug -> print cycleTime en calculationTime en PID values etc.
void onDebug() {
  SerialPort.print(F("cycle time (micros): "));
  SerialPort.println(params.cycleTime);

  SerialPort.print(F("max calculation time (micros): "));
  SerialPort.println(calculationTime);
  calculationTime = 0; // reset max

  SerialPort.print(F("baseSpeed: ")); SerialPort.println(params.baseSpeed);
  SerialPort.print(F("maxSpeed: ")); SerialPort.println(params.maxSpeed);
  SerialPort.print(F("Kp: ")); SerialPort.println(params.Kp, 6);
  SerialPort.print(F("Ki: ")); SerialPort.println(params.Ki, 6);
  SerialPort.print(F("Kd: ")); SerialPort.println(params.Kd, 6);
  SerialPort.print(F("invertLeft: ")); SerialPort.println(params.invertLeft ? "on":"off");
  SerialPort.print(F("invertRight: ")); SerialPort.println(params.invertRight ? "on":"off");
  SerialPort.print(F("sensorInvert: ")); SerialPort.println(params.sensorInvert ? "on":"off");

  BT.print(F("baseSpeed: ")); BT.println(params.baseSpeed);
  BT.print(F("maxSpeed: ")); BT.println(params.maxSpeed);
  BT.print(F("Kp: ")); BT.println(params.Kp, 6);
  BT.print(F("Ki: ")); BT.println(params.Ki, 6);
  BT.print(F("Kd: ")); BT.println(params.Kd, 6);
  BT.print(F("invertLeft: ")); BT.println(params.invertLeft ? "on":"off");
  BT.print(F("invertRight: ")); BT.println(params.invertRight ? "on":"off");
  BT.print(F("sensorInvert: ")); BT.println(params.sensorInvert ? "on":"off");
}

// status -> print huidige status en laatste sensorwaarden
void onStatus() {
  onDebug();
  SerialPort.println(F("Last sensor values (0..1000):"));
  // update sensor values once for reporting (niet in cyclische lus)
  readCalibratedSensors();
  for (int i=0;i<NUM_SENSORS;i++) {
    SerialPort.print(F("S")); SerialPort.print(i); SerialPort.print(F(": "));
    SerialPort.println(sensorValues[i]);
  }
}

// motortest <side:left|right|both> <speed:-255..255> <duration_ms>
void onMotorTest() 
{
  if (!activeCmd) return;  // extra safety

  char* side = activeCmd->next();   
  char* sspd = activeCmd->next();   
  char* sdur = activeCmd->next(); 

  if (!side || !sspd) 
  {
    SerialPort.println(F("motortest <left|right|both> <speed> [duration_ms]"));
    BT.println(F("motortest <left|right|both> <speed> [duration_ms]"));
    return;
  }
  int speed = atoi(sspd);
  unsigned long duration = 1000;
  if (sdur) duration = atol(sdur);

  SerialPort.println(F("Starting motor test..."));
  BT.println(F("Starting motor test..."));
  if (strcmp(side,"left")==0 || strcmp(side,"both")==0) 
  {
    setMotorA(params.invertLeft ? -speed : speed);
  }
  if (strcmp(side,"right")==0 || strcmp(side,"both")==0) 
  {
    setMotorB(params.invertRight ? -speed : speed);
  }
  delay(duration);
  // stop motors
  setMotorA(0);
  setMotorB(0);
  SerialPort.println(F("Motor test finished."));
  BT.println(F("Motor test finished."));
}

// sensortest -> print ruwe analoge waarden en gekalibreerde waarden
void onSensorTest() {
  SerialPort.println(F("Sensor raw / calibrated:"));
  for (int i=0;i<NUM_SENSORS;i++){
    int raw = analogRead(sensorPins[i]);
    int minv = params.sensorMin[i];
    int maxv = params.sensorMax[i];
    int cal;
    if (maxv > minv) cal = map(raw, minv, maxv, 0, 1000);
    else cal = map(raw, 0, 1023, 0, 1000);
    cal = constrain(cal, 0, 1000);
    if (params.sensorInvert) cal = 1000 - cal;
    SerialPort.print(F("S")); SerialPort.print(i); SerialPort.print(F(" raw="));
    SerialPort.print(raw); SerialPort.print(F(" cal=")); SerialPort.println(cal);
  }
}



// help -> print commands
void onHelp() 
{
  SerialPort.println(F("Commands:"));
  SerialPort.println(F("  set <param> <value>    (kp,ki,kd,base,max,cycle,invertLeft,on/off,invertRight,on/off,sensorInvert,on/off)"));
  SerialPort.println(F("  calibrate [ms]         (kalibratie sensoren, default 2000ms)"));
  SerialPort.println(F("  debug                  (print cycle and calculation times + PID)"));
  SerialPort.println(F("  status                 (status + sensor values)"));
  SerialPort.println(F("  save                   (save params to EEPROM)"));
  SerialPort.println(F("  load                   (load params from EEPROM)"));
  SerialPort.println(F("  motortest <left|right|both> <speed> [ms]"));
  SerialPort.println(F("  sensortest             (print raw and calibrated sensor values)"));
  SerialPort.println(F("  help                   (this list)"));
  SerialPort.println(F("  start                  (starts the lineFollower)"));
  SerialPort.println(F("  stop                   (stops de lineFollower)"));

  BT.println(F("Commands:"));
  BT.println(F("  set <param> <value>    (kp,ki,kd,base,max,cycle,invertLeft,on/off,invertRight,on/off,sensorInvert,on/off)"));
  BT.println(F("  calibrate [ms]         (kalibratie sensoren, default 2000ms)"));
  BT.println(F("  debug                  (print cycle and calculation times + PID)"));
  BT.println(F("  status                 (status + sensor values)"));
  BT.println(F("  save                   (save params to EEPROM)"));
  BT.println(F("  load                   (load params from EEPROM)"));
  BT.println(F("  motortest <left|right|both> <speed> [ms]"));
  BT.println(F("  sensortest             (print raw and calibrated sensor values)"));
  BT.println(F("  help                   (this list)"));
  BT.println(F("  start                  (starts the lineFollower)"));
  BT.println(F("  stop                   (stops de lineFollower)"));
}

// default handler
void onUnknownCommand(char *command) 
{
  SerialPort.print(F("unknown command: \""));
  SerialPort.print(command);
  SerialPort.println(F("\""));
  BT.print(F("unknown command: \""));
  BT.print(command);
  BT.println(F("\""));

  SerialPort.print("ASCII: ");
  BT.print("ASCII: ");
  for (int i = 0; command[i]; i++) {
    SerialPort.print((int)command[i]);
    BT.print((int)command[i]);
    SerialPort.print(" ");
    BT.print(" ");
  }
  SerialPort.println();
  BT.println();
}


// START commando
void onStart() {
  running = true;
  SerialPort.println("Robot started");
  BT.println("Robot started");
}


// STOP commando
void onStop() {
  running = false;
  setMotorA(0);
  setMotorB(0);
  SerialPort.println("Robot stopped");
  BT.println("Robot stopped");
}

