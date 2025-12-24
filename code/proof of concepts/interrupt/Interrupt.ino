void loop() 
{
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