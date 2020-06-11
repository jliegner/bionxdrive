# bionxdrive
Demo Code um einen BionX Motor zu Drehen und Bremsen ohne weiter BionX Componenten

Um einen BionX Fahrradmotor zum Drehen/Bremsen zu bewegen ist folgendes nötig:

1. Spannungsversorgung Motor Power (24-48V)
Die Hauptversorgung muss von einem Akku kommen und es muss sichergestellt werden, dass während des Bremsens/Rekuperation die Verbindung zum Akku nicht getrennt wird. Anderenfalls kann die Spannung hochlaufen und den motor sehr schnell zerstören.

2. 12V fuer den Motor Elektronik
Gemessen zieht der Motor ca. 200-250mA Strom auf der 12V Schiene. Das muss sicher bereitgestellt werden.

3. 5V für die eigene Elektronik

Verwendet wird ein "BluePill"-Board mit angeschlossenem CAN-Transceiver. Ein Poti an Port A0 (0-3.3V) und ein Taster an A1 gegen GND.

