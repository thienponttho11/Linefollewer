# Gebruiksaanwijzing

### opladen / vervangen batterijen
Er wordt een 2S-lipo 3000mAh 15C batterij gebruikt, deze batterij kan je opladen a.d.h.v. een oplaadkabel en moet dus niet verwijdert worden van de linefollower hiervoor.

### draadloze communicatie
#### verbinding maken
De robot is uitgerust met een HC-05 Bluetooth module die seriële communicatie mogelijk maakt met een laptop of smartphone.
Ik gebruikte de app Serial Bluetooth Terminal op de smartphone. Na het inschakelen van uw bluetooth en de robot verschijnt de HC-05 als Bluetooth-apparaat in de app en kan je connecteren.
Na koppeling kan men commando’s versturen en feedback ontvangen.
--> de robot kan ook via USB (Serial Monitor) bediend worden. Beide interfaces werken gelijktijdig en gebruiken dezelfde commando’s.

#### commando's
start
stop
help
status
debug
sensortest
motortest <left|right|both> <speed> [ms]
calibrate [ms]

set kp <value>
set ki <value>
set kd <value>
set base <value>
set max <value>
set cycle <value>
set invertLeft on|off
set invertRight on|off
set sensorInvert on|off

### kalibratie
Tijdens de kalibratie worden voor elke sensor de minimum- en maximumwaarden opgeslagen (sensorMin en sensorMax), deze krijg je door na bv. de commando calibrate 5000 in te voeren, binnen deze 5sec de robot met de sensoren van het witte deel over de zwarte lijn van het parcour te bewegen.
Deze waarden worden dan gebruikt om de ruwe analoge sensormetingen om te zetten naar een genormaliseerde schaal van 0 tot 1000.

### settings
De robot rijdt stabiel met volgende parameters:  
--> cycleTime ≈ 10 ms
--> baseSpeed = 175
--> maxSpeed = 215
--> Kp = 1.35
--> Ki = 0.0
--> Kd = 0.45

### start/stop button
De robot gebruikt geen fysieke start/stop-knop.
Starten en stoppen gebeurt uitsluitend via softwarecommando’s (start en stop) die verstuurd worden via USB (Serial Monitor) of Bluetooth, de robot stopt uiteraard ook wel wanneer je de I/O knop schakelt maar dat zet alles direct onder spanning.
