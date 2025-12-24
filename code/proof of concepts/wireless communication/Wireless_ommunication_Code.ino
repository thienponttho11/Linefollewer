// SerialCommand object
SoftwareSerial BT(2, 3); // RX, TX HC-05
SerialCommand sCmdUSB(SerialPort); // voor USB monitor
SerialCommand sCmdBT(BT); // voor App monitor

SerialCommand* activeCmd = nullptr;

void setup() 
{
  // Serial + commands
  SerialPort.begin(Baudrate);
  BT.begin(9600); 
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
}