# start/stop interrupt proof of concept

De start() functie zet running = true, initialiseert interne PID-waarden indien nodig en laat de cyclische stepCycle() opnieuw actief worden. 

De stop() functie zet running = false en stopt de motoren onmiddellijk door setMotorA(0) en setMotorB(0) aan te sturen. Hierdoor wordt de volledige line-following routine veilig onderbroken, nog vóór er nieuwe sensormetingen of PID-berekeningen plaatsvinden.
