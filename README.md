# bionxdrive
Demo Code um einen BionX Motor zum Drehen und Bremsen zu bewegen 
ohne weiter BionX Componenten.

Um einen BionX Fahrradmotor zum Drehen/Bremsen zu bewegen ist folgendes nötig:

1. Spannungsversorgung Motor Power (24-48V).
Die Hauptversorgung muss von einem Akku kommen und es muss sichergestellt werden, 
dass während des Bremsens/Rekuperation die Verbindung zum Akku nicht getrennt wird. 
Anderenfalls kann die Spannung hochlaufen und den Motor sehr schnell zerstören.

2. 12V fuer den Motor Elektronik
Gemessen zieht der Motor ca. 200-250mA Strom auf der 12V Schiene. 
Das muss sicher bereitgestellt werden.

3. 5V für die eigene Elektronik

Verwendet wird ein "BluePill"-Board mit angeschlossenem CAN-Transceiver an PB8 und PB9. 
Ein Poti an Port PA0 (0-3.3V) und ein Taster an PA1 gegen GND.

Mit dem Poti kann man ein Drehmoment vorgeben, Mittelstellung ist 0. 
Solange der Taster gedrückt wird, wird das eingestelte Drehmoment an den Motor als Sollwert übertragen,
bei losgelassenem Taster wird 0 als Sollwert gesendet.

Das Projekt ist für Segger Embedded Studio:
https://www.segger.com/downloads/embedded-studio/

Alternativ kann man das auch nur mit der gnuarm-Toolchain und buildgnu.bat übersetzen. 
Unter Windows verwende ich die windows-build-tools:
https://github.com/gnuarmeclipse/windows-build-tools/releases
für make u.s.w. 
gnuarmeclipse-build-tools-win64-xxx-setup.exe 
kann man mit die mit 7zip öffnen und die binaries aus bin herauskopieren.

 

 

