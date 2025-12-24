# H-Bridge proof of concept

De werking van de H-bridge wordt bewezen door de software die twee DC-motoren volledig onafhankelijk aanstuurt via de functies setMotorA() en setMotorB(). 

De snelheid van elke motor is traploos regelbaar doordat de code een variabele PWM-waarde gebruikt (speed), die elke waarde binnen het bereik accepteert en daardoor een lineaire snelheidsregeling mogelijk maakt. De draairichting wordt eveneens softwarematig bepaald: een positieve snelheid stuurt de motor voorwaarts, terwijl een negatieve snelheid de H-bridge automatisch in tegenrichting schakelt. 

Het commando motortest <left|right|both> <speed> [ms] toont aan dat zowel de linker als rechter motor afzonderlijk of samen kunnen worden getest met elke gewenste snelheid en draairichting. Hiermee voldoet het project aan alle eisen van het proof-of-concept: minimale hardware, onafhankelijke sturing, volledige richtingscontrole en traploze snelheidsregeling.
