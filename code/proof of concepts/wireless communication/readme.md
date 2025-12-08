# draadloze communicatie proof of concept
minimale hard- en software waarmee aangetoond wordt dat duplex kan gecommuniceerd worden tussen de microcontroller en een smartphone, gebruik makend van Serial Bluetooth Terminal
Minimale hardware, Bekabeling Arduino ↔ HC-05;

HC-05 VCC → 5V
HC-05 GND → GND
HC-05 TXD → Arduino pin 2
(via SoftwareSerial RX)
HC-05 RXD → Arduino pin 3
(via SoftwareSerial TX
De code bevat: SoftwareSerial BT(2, 3); // RX, TX (Arduino)

Minimale software;

BT.begin(9600);
if (BT.available()) 
{
    activeCmd = &sCmdBT;
    sCmdBT.readSerial();
}

### configuratie
HC-05: standaardinstellingen
Baudrate: 9600
PIN: 1234 of 0000
Naam: HC-05

App: Serial Bluetooth Terminal
Instellingen:
Baudrate = 9600
Newline = \n
Character echo = optioneel aan
Emulation mode = terminal

Arduino sketch:
USB op 115200 baud (debug)
Bluetooth op 9600 baud (HC-05 standaard)
Commands worden automatisch via beide lijnen verwerkt.

### opmerkingen
- Tijdens uitvoering maakt het systeem geen onderscheid tussen USB en Bluetooth; beide interfaces gebruiken hetzelfde commando- en responsesysteem.
- De Bluetooth-interface is non-blocking en volledig geïntegreerd in de main loop.
- Debug-uitvoer kan zowel via USB als via Bluetooth uitgevoerdt worden en bekeken.
- Duplex betekent niet dat TX/RX exact gelijktijdig lopen, maar dat beide richtingen onafhankelijk functioneren zonder mode-switching.

### gebruiksaanwijzing
1. Pairen
- Open Bluetooth-instellingen op de smartphone
- Zoek naar HC-05
- Pair met PIN 1234

3. Open de app Serial Bluetooth Terminal
- Verbind met HC-05
- Terminal opent automatisch

5. Nu test de duplex communicatie met commando's
