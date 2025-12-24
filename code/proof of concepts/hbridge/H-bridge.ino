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