# Sensoren proof of concept

Minimale hardware;
als u de volgende declaratie bekijkt zie je al dat elke sensor apart binnen komt in de microcontroller als 8 onafhankelijke ADC-kanalen
const uint8_t sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};

Minimale Software;
void onSensorTest() 
{
  SerialPort.println(F("Sensor raw / calibrated:"));
  for (int i=0;i<NUM_SENSORS;i++)
  {
    int raw = analogRead(sensorPins[i]);   // <-- pure ADC, 0..1023
  }
}

Bij sensortest worden de ruwe ADC-waarden zonder calibratie, normalisatie of interpolatie uitgelezen en elke sensor wordt 100% onafhankelijk uitgelezen
en dit kan je zien door de commando "sensortest" uit te voeren wanneer het programma aan het runnen is, dan krijg je iets zoals dit:
S0 raw=312
S1 raw=745
S2 raw=1023
S3 raw=489
S4 raw=50
S5 raw=0
S6 raw=820
S7 raw=234

Dit systeem demonstreert dat 6 sensoren onafhankelijk worden uitgelezen met volledig ADC-bereik (0–1023), zonder enige vorm van calibratie, normalisatie of interpolatie.
