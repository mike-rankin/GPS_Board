On a Windows 7 machine place the contents on libraries.zip into a temp file on your computer.
Install Arduino 1.0.6

Place the TinyGPS library in your Arduino/libraries folder. On my PC its  MyDocuments/Arduino/libraries
Place the PinChangeInt library in your Arduino/libraries folder. On my PC its  MyDocuments/Arduino/libraries

In the same place as where the GPS_Cube.ino sketch is, place data.h, EEPROMAnything.h, mySoftwareSerial.cpp and mySoftwareSerial.h all in the same foilder.

Plug in the GPS Board and hopefully the FTDI chip driver will auto install, a new com port will appear
Pick Tools,Board Arduino Pro or Pro Mini
Pick Tools, Processor, ATmega328 (3.3v, 8 Mhz)
Verify in the IDE by viewing in Tools, Port does indeed show the new com port

Then in the IDE open the GPS_Cube.ino file. If it compiles error free then you are good to upload the sketch.
