
//Modified print_coord(); to prevent locking up in certain countries

//==========================================================//
//                                                          //
//    GPS Toy By KaR]V[aN, http://karman.homelinux.net      //
//    Thanks to the Arduino comunity for providing code,    //
//    libraries and platforms. Also many thanks to Nitz,    //
//    whose coding skills helped so much.                   //
//                                                          //
//==========================================================//

/* 
 * This is a simple GPS Toy/'on board computer' that shows GPS
 * data in a tiny 128x64 OLED display (I2C driven, 0.96").
 * Because being I2C display the frame rate is very slow 
 * but saves pins for future use.
 */
 
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include "mySoftwareSerial.h" 
#include <TinyGPS.h>
#include <Wire.h>
#include <avr/pgmspace.h>

// So, why use a custom software serial? This addresses a problem sharing 
// interrupts between pinchangeint and software serial, by disabling the 
// unsued ports on both of them. Meanwhile pinchangeint allows easy disabling,
// newsoftwareserial doesn't, so have to comment code. More info:
// http://code.google.com/p/arduino-pinchangeint/issues/detail?id=7

//---------------FONT + GRAPHIC-----------------------------//
#include "data.h"
//==========================================================//

// OLED I2C bus address
#define OLED_address  0x3c

// Setup the GPS MT3339 receiver
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" // turn on GPRMC and GPGGA —– TinyGPS only uses these two, all others can be turned off
#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F\r\n" // Every 1000ms (1Hz)
#define PMTK_SET_NMEA_UPDATE_2HZ "$PMTK220,500*2B\r\n" // Every 500ms (2Hz)
#define PMTK_SET_NMEA_BAUDRATE "$PMTK251,4800*14\r\n" // Baudrate to 4800
#define PMTK_API_SET_SBAS_ENABLED "$PMTK313,1*2E"   // Enable SBAS sattelite for fast fix
#define PMTK_API_SET_SBAS_Mode "$PMTK319,1*24"      // Integrity mode

// I'll attach interrupt to digital pins 4 and 5, both 
// on port D. Both ports B and C are to be disabled:
#define NO_PORTB_PINCHANGES // to indicate that port b will not be used for pin change interrupts
#define NO_PORTC_PINCHANGES // to indicate that port c will not be used for pin change interrupts
#include <PinChangeInt.h>

TinyGPS gps;
SoftwareSerial nss(9, 8);
static bool feedgps();
const int buttonA = 4;
const int buttonB = 5;
unsigned int buttonStateB = 0;      
unsigned int buttonStateA = 0;       
unsigned int mode=0;
bool course_degree = false;
bool control_blink = false;
bool control_clock = false;
bool interrupt = false;
bool doing_menu = false;
float SAVED_LAT = 0.0, SAVED_LON = 0.0, LAST_LAT = 0.0, LAST_LON = 0.0;
unsigned int TZ = 2;
unsigned long int tripDistance = 0;
static const byte settings_version = B10000011;
bool fix_first = 1;
float last_course = 0.0;
int TMP75_Address = 0x49;
float miniTimer = millis();
bool debug = 0; // turns on GPS data over USB serial



//---------------Saved Data Map-----------------------------//
// Bytes  Size  Purpose
// ····························································
// 0      1     Settings Version
// 1-4    4     Saved Latitude
// 5-8    4     Saved Longitude
// 9-10   2     Saved Mode
// 11     1     Saved Course preference (Degrees - Cardinal)
// 12     1     Saved Clock prefence (12h - 24h)
// 13-14  2     Saved Time Zone
// 15-18  4     Saved trip distance
//==========================================================//


void setup()
{
  // Load previous Settings
  byte read_settings_version;
  EEPROM_readAnything(0, read_settings_version);
 
  if(settings_version == read_settings_version) // If version matchs, load vars from EEPROM
  {  
    EEPROM_readAnything(1, SAVED_LAT);
    EEPROM_readAnything(5, SAVED_LON);
    EEPROM_readAnything(9, mode);
    EEPROM_readAnything(11, course_degree);
    EEPROM_readAnything(12, control_clock);
    EEPROM_readAnything(13, TZ);
    EEPROM_readAnything(15, tripDistance);
  }
  else                                          // If not, initialize with defaults
  {
    EEPROM_writeAnything(0, settings_version);
    EEPROM_writeAnything(1, SAVED_LAT);
    EEPROM_writeAnything(5, SAVED_LON);
    EEPROM_writeAnything(9, mode);
    EEPROM_writeAnything(11, course_degree);
    EEPROM_writeAnything(12, control_clock);
    EEPROM_writeAnything(13, TZ);
    EEPROM_writeAnything(15, tripDistance);
  }
  
  // Buttons...
  pinMode(buttonA, INPUT);
  pinMode(buttonB, INPUT);    
  digitalWrite(buttonA, LOW);
  digitalWrite(buttonB, LOW);
  
  // Attach interrupts to both buttons on rising change
  PCintPort::attachInterrupt(buttonA, &interrupt_button, RISING); 
  PCintPort::attachInterrupt(buttonB, &interrupt_button, RISING); 
 
  // Initialize GPS receiver
  // nss.begin(4800); // Old GPS EM411
  // Init in 9600 just in case MT3339 lost its settings
  nss.begin(9600);
  while (!nss.available()){} // yeah I know, ugly way to empty buffer...
  nss.print(PMTK_SET_NMEA_BAUDRATE); // baudrate to 4800

  // .. then delay 10ms and re-initialize in 4800
  delay(10);  
  nss.begin(4800);   
  while (!nss.available()){} // ...
  nss.print(PMTK_SET_NMEA_OUTPUT_RMCGGA); // $GPxxx Messages
  nss.print(PMTK_SET_NMEA_UPDATE_1HZ); // messages 2 times a second
  nss.print(PMTK_API_SET_SBAS_ENABLED); // enable SBAS fixing
  nss.print(PMTK_API_SET_SBAS_Mode); // integrity mode for SBAS
 
  // Initialize I2C and OLED Display
  Wire.begin();
  init_OLED();
 
  // Initialize TMP75
  Wire.beginTransmission(TMP75_Address);       // Address the TMP75 sensor
  Wire.write(0x01);                       // Address the Configuration register 
  Wire.write(B01100000);                         // Set the temperature resolution 
  Wire.endTransmission();                      // Stop transmitting
  Wire.beginTransmission(TMP75_Address);       // Address the TMP75 sensor
  Wire.write(0x00);                          // Address the Temperature register 
  Wire.endTransmission();                      // Stop transmitting 

  reset_display();           // Clear logo and load saved mode
  nss.listen();
  if (debug)
    Serial.begin(115200);
}


//==========================================================//
void loop()
{  
  // Read Button state
  if (buttonStateA == HIGH || buttonStateB == HIGH)
  {
    doing_menu = true;                            // we are in button fuctions, ignore interrupt bool
    unsigned long button_delay = millis();
    
    while (millis() - button_delay < 50) 
    {
      if (buttonStateA == LOW) 
        buttonStateA = digitalRead(buttonA);
      if (buttonStateB == LOW) 
        buttonStateB = digitalRead(buttonB);
    }
      
    // Both buttons pushed
    if (buttonStateB == HIGH && buttonStateA == HIGH)
    {
      displayOff();
      setXY(0,0);
  
      for(int i=0;i<128*8;i++)     // show 128* 64 Logo
        SendChar(pgm_read_byte(logo+i));
    
      displayOn();
      sendStrXY("Carlos Sancho",7,2);
      sendStrXY("v1.1", 0, 12);
      delay(5000);
      clear_display();
      sendStrXY("Hardware Design by Mike Rankin",0,0);
      sendStrXY("Using TinyGPS by Mikal Hart",3,0);
      sendStrXY("http://karman.cc",6,0);
      delay(5000);
      reset_display();
    }
    
    // Menu/mode button
    else if(buttonStateA == HIGH && buttonStateB == LOW) 
    {
      // Change mode
      if (mode == 0) 
        mode = 1;
      
      else if (mode == 1) 
        mode = 2;
        
      else if (mode == 2)
        mode = 3;
        
      else if (mode == 3)
        mode = 0;
      
      EEPROM_writeAnything(9, mode); // Save mode
      reset_display();
    } 
    
    // Other button
    else if (buttonStateB == HIGH && buttonStateA == LOW) 
    {
      reset_buttons();
      
      // Change cardinal <-> degrees
      if (mode == 0) 
      {
        course_degree = !course_degree;
        EEPROM_writeAnything(11, course_degree);
        reset_display();
      }
      
      // Memorize current location after 3 seconds pushing button
      else if (mode == 1) 
      {
        displayOff();
        clear_display();
        sendStrXY("Memorizing in: ", 0, 0);
        printBigNumber('3', 3 ,6);
        displayOn();
        boolean exit = 0;
        unsigned long start = millis();
        while (digitalRead(buttonB) == HIGH && !exit)
        {          
          if (millis() - start > 3000) 
          {
            clear_display();
            exit = true;
            sendStrXY("Coordinates memorized:", 0, 0);
            saveActualLoc(gps);
            delay(2000);
          } 
          
          else if (millis() - start > 2000)
            printBigNumber('1', 3 ,6);
            
          else if (millis() - start > 1000)
            printBigNumber('2', 3 ,6);
        }
        reset_display();
      }
      
      // Reset trip 
      else if (mode == 2)
      {
        displayOff();
        clear_display();
        sendStrXY("Clearing in: ", 0, 0);
        printBigNumber('3', 3 ,6);
        displayOn();
        boolean exit = 0;
        unsigned long start = millis();
        while (digitalRead(buttonB) == HIGH && !exit)
        {          
          if (millis() - start > 3000) 
          {
            clear_display();
            exit = true;
            unsigned long age;
            float flat, flon;
            gps.f_get_position(&flat, &flon, &age);
            sendStrXY("Trip reseted", 0, 0);
            tripDistance = 0;
            LAST_LAT = flat;
            LAST_LON = flon;
            EEPROM_writeAnything(15, tripDistance);
            delay(2000);
          } 
          
          else if (millis() - start > 2000)
            printBigNumber('1', 3 ,6);
            
          else if (millis() - start > 1000)
            printBigNumber('2', 3 ,6);
        }
        reset_display();
      }
      
      // Change clock hour mode, 12h <-> 24h in short push (< 3 secs)
      // In long push (> 3 sec) enters in time zone setting mode
      else if (mode == 3)
      {
        reset_buttons();
        boolean exit = 0;
        boolean mode_timezone = false;
        unsigned long start = millis();
        while (digitalRead(buttonB) == HIGH && !exit)
        {
          if (millis() - start > 3000) 
          {
            exit = true;
            mode_timezone = true;
          }
        }
        
        if (mode_timezone)
        {
          displayOff();
          clear_display();
          sendStrXY("Select TimeZone:", 0, 0);
          sendStrXY("Menu to set", 7, 2);
          displayOn();
          reset_buttons();
          bool print_TZ = true;
          while(buttonStateA != HIGH)
          {
            
            if (buttonStateB == HIGH) 
            {
              // this loop avoids false contacts of defective switches and sparks
              while (digitalRead(buttonB) != LOW)
                delay(10); 
              
              buttonStateB = LOW;
              TZ++;
              print_TZ = true;
            }
            
            if (TZ == 13)
              TZ = -11;
            
            // Avoid to print always same value
            if (print_TZ) {
              print_timezone(TZ);
              print_TZ = false;
            }
            
            delay(10); // to not fall into an endless loop. 
          }
          EEPROM_writeAnything(13, TZ);
        }
        else 
        {
          control_clock = !control_clock;
          EEPROM_writeAnything(12, control_clock);
        }
        reset_display();
      }
    }
    reset_buttons();
    doing_menu = false;    // Reset both vars on buttons exit
    interrupt = false;
  }
  
  
  // Update display
  updateScreen(gps);  
     
  // update trip distance
  udpateTrip(gps);
}

// Change button state on interrupt
void interrupt_button (void)
{
  if (PCintPort::arduinoPin == buttonA)
    buttonStateA = HIGH;
  else if (PCintPort::arduinoPin == buttonB)
    buttonStateB = HIGH;
    
  interrupt = true;          // Stop printing on display, speeds up response.
}

// Resets button states
static void reset_buttons(void)
{
  buttonStateA = LOW;
  buttonStateB = LOW;
}

//==========================================================//
// Resets display depending on the actual mode.
static void reset_display(void)
{
  displayOff();
  clear_display();
  if (mode == 0)
  {
    printKmh_unit();
  }
  else if (mode == 1) 
  {
    //nothing here by now
  } 
  else if (mode == 2) 
  {
    sendStrXY("Trip sum:", 0, 3);
  }
  else if (mode == 3)
  {
  //  sendStrXY("Atomic GPS Watch",0,0);
  }  
  updateScreen(gps);
  displayOn();
}

//==========================================================//
// Updates screen depending on the actual mode.
static void updateScreen(TinyGPS &gps) 
{
  unsigned long age;
  float flat, flon;
  gps.f_get_position(&flat, &flon, &age);
  
  if (mode == 0)
  {
    print_date(gps);
    printAlt(gps.f_altitude(), 0, 0);
    printSpeed(gps.f_speed_kmph());
    printSats(gps.satellites(), 0, 10, age);
    printCourse(gps); 
  }  
  else if (mode == 1)
  { 
    print_coord(flat, TinyGPS::GPS_INVALID_F_ANGLE, 0, 0);
    print_coord(flon, TinyGPS::GPS_INVALID_F_ANGLE, 0, 9);
    sendDistanceBetwen(gps);
    printCourse(gps);
    print_coord(SAVED_LAT, TinyGPS::GPS_INVALID_F_ANGLE, 7, 0);
    print_coord(SAVED_LON, TinyGPS::GPS_INVALID_F_ANGLE, 7, 9);
  }
  else if (mode == 2)
  {
    printTrip();
    printTemp(7,0);
    printSats(gps.satellites(), 7, 10, age);
  }
  else if (mode == 3)
  {
    print_date(gps);
    printTemp(0,0);
    printSats(gps.satellites(), 0, 10, age);
  }
}



//==========================================================//
// Shows and saves actual coordinates
static void saveActualLoc(TinyGPS &gps)
{
  unsigned long age;
  float flat, flon;
 
  gps.f_get_position(&flat, &flon, &age);
  sendStrXY("Lat: ", 4, 0);
  sendStrXY("Lon: ", 5, 0);
  print_coord(flat, TinyGPS::GPS_INVALID_F_ANGLE, 4, 5);
  print_coord(flon, TinyGPS::GPS_INVALID_F_ANGLE, 5, 5);
  SAVED_LAT = flat;
  SAVED_LON = flon;
  EEPROM_writeAnything(1, SAVED_LAT);
  EEPROM_writeAnything(5, SAVED_LON);
}

//==========================================================//
// Prints display trip update
static void printTrip () {
 /* char sz[32];
  sprintf(sz, "RAW: %d", tripDistance);
  sendStrXY(sz, 7, 0);*/
  print_distance(tripDistance);
}

//==========================================================//
// Gets current location and if > 20, sums to trip global var.
static void udpateTrip (TinyGPS &gps) {
  unsigned long age;
  float flat, flon;
  int max_dist = 25;
 
  gps.f_get_position(&flat, &flon, &age);
  if (flat == TinyGPS::GPS_INVALID_F_ANGLE)
    return;
  else {
    
    int mySpeed = (int)gps.f_speed_kmph();
    
    if(mySpeed > 5) 
      max_dist = 20;
    if(mySpeed > 10) 
      max_dist = 15;
    if(mySpeed > 15) 
      max_dist = 5;
    
    if(fix_first) 
    {
      LAST_LAT = flat;
      LAST_LON = flon;
      fix_first = false;
      last_course = TinyGPS::course_to(flat, flon, LAST_LAT, LAST_LON);
    } 
    else 
    {
    
      // Distance in meters
      unsigned long int dist = (long int)TinyGPS::distance_between(flat, flon, LAST_LAT, LAST_LON);

      if (dist > max_dist)   
      {
        
        // Try some course correction, props to Nitz
        float actual_course = TinyGPS::course_to(flat, flon, LAST_LAT, LAST_LON);
        float new_course = last_course - actual_course;
        new_course -= new_course / 2;
       
        tripDistance += dist * fabs((float)cos(new_course));
        
        last_course = actual_course;
        LAST_LAT = flat;
        LAST_LON = flon;
        EEPROM_writeAnything(15, tripDistance);
      }
    }   
  }
}

//==========================================================//
// Prints actual course or course to a point (distance mode) 
// in cardinal or dregree mode.
static void printCourse(TinyGPS &gps) 
{
  unsigned long age;
  float flat, flon;
  gps.f_get_position(&flat, &flon, &age);
  
  if (flat == TinyGPS::GPS_INVALID_F_ANGLE) 
    sendStrXY("C:N/A", 2, 10);
  else
  {
    char sz[32];
    if(course_degree)       // Print course in degrees mode
    {
      char sy[10];
      
      if (mode == 0)
        dtostrf(gps.f_course(),3,0,sy);
      else if (mode == 1)
        dtostrf(TinyGPS::course_to(flat, flon, SAVED_LAT, SAVED_LON),3,0,sy);
        
      sprintf(sz, "C:%s", sy);
      
      setXY(2, 15);
      for(int i=0;i<8;i++)     // print degree simbol
        SendChar(pgm_read_byte(myDregree+i));
    }    
    else                // Print course in cardinal mode
    {
      char tmp[3];
     
      if (mode == 0)
        sprintf(tmp, "%s", TinyGPS::cardinal(gps.f_course()));
      else if (mode == 1)
        sprintf(tmp, "%s", TinyGPS::cardinal(TinyGPS::course_to(flat, flon, SAVED_LAT, SAVED_LON)));
        
      int lon = strlen(tmp);
 
      sprintf(sz, "C:% 3s", tmp);
    }
      
    sendStrXY(sz, 2, 10);
  }
}


//==========================================================//
// Prints distance between two points given by actual and saved coordinates.
// To do so, function gets the two mayor magnitudes and prints them.
static void sendDistanceBetwen(TinyGPS &gps) 
{
  unsigned long age;
  float flat, flon;
  //char unit[10] = "m ";
 // char sz[32];
 // char next_unit[2];
 
  gps.f_get_position(&flat, &flon, &age);
  if (flat == TinyGPS::GPS_INVALID_F_ANGLE)
  {
    //sendDistance("000", " m");
    sendSpeed("000");
    sendStrXY("m ", 5, 9);
    sendStrXY("000*", 5, 12);
  }
  else {
    // Distance in meters
    unsigned long int dist = (long int)TinyGPS::distance_between(flat, flon, SAVED_LAT, SAVED_LON);
    print_distance(dist);
 
  }
}

//==========================================================//
// Prints distance.
static void print_distance (unsigned long int dist)
{
   char unit[10] = "m ";
   char sz[32];
   char next_unit[2];
   char tmp[32];
   unsigned long int little_ones;
   unsigned long int big_ones;
   
   if (dist >= 1000000000) 
    {
      little_ones = (dist % 1000000000) / 1000000;
      big_ones = (dist - (little_ones * 1000000)) / 1000000000;
      sprintf(next_unit, "%s", "M");
      sprintf(unit, "%s", "Gm");
    }
    else if (dist >= 1000000) 
    {
      little_ones = (dist % 1000000) / 1000;
      big_ones = (dist - (little_ones * 1000)) / 1000000;
      sprintf(next_unit, "%s", "K");
      sprintf(unit, "%s", "Mm");
    }
    else if (dist >= 1000) 
    {
      little_ones = dist % 1000;
      big_ones = ((dist - little_ones) / 1000);
      sprintf(next_unit, "%s", "m");
      sprintf(unit, "%s", "Km");
    }
    else
    {
        big_ones = dist;
        little_ones = 0;
        sprintf(next_unit, "%s", "*");
        sprintf(unit, "%s", "m ");
    }
      
    sprintf(sz, "%03d", big_ones);
      
    sprintf(tmp, "%03d", little_ones);
          
  //  sendDistance(sz, unit);
    sendSpeed(sz);
    sendStrXY(unit, 5, 9);

    sendStrXY(tmp, 5, 12);
    sendStrXY(next_unit, 5, 15);
}


//==========================================================//
// Prints current sattelites being used by GPS module.
static void printSats(int sats, int X, int Y, int age)
{
  if (sats == TinyGPS::GPS_INVALID_SATELLITES || age > 3000)
    sendStrXY("NO SIG", X, Y);
  else {
    char sz[7];
    (sats>9)?sprintf(sz, "%d SAT", sats):sprintf(sz, "0%d SAT", sats);
    sendStrXY(sz, X, Y);
  }
}


//==========================================================//
// Prints altitude adjusting it to display representation.
static void printAlt(float altitud, int X, int Y)
{
  if (altitud == TinyGPS::GPS_INVALID_F_ALTITUDE)
    sendStrXY("00000 m", X, Y);
  else {
    char sz[32];
    int alt = (int)altitud;
    
    if (alt<0)
      sprintf(sz, "-%04d m", abs(alt));
    else
      sprintf(sz, "%05d m", alt);
      
    sendStrXY(sz, X, Y);
  }
}


//==========================================================//
// Prints Speed and it's first decimal adjusting 0 in the left.
static void printSpeed(float val)
{
  if(val == TinyGPS::GPS_INVALID_F_SPEED)
  {
      sendSpeed("000");
      sendStrXY(".0",5,14);
  }
  else {
    char sx[5];
    char sy[5];
    char sz[5];
    int num = (int)val;
    float tmp = val - num;
    
    dtostrf(tmp,3,1,sy);
    sprintf(sx, ".%c", sy[2]);
    sprintf(sz, "%03d", num);
   
    sendSpeed(sz);
    sendStrXY(sx,5,14);
  }
}

//Possibly Locking up Board
/*
//==========================================================//
// Prints a floating coordinate yanking last two decimals for display
// fitting.
static void print_coord(float val, float invalid, int X, int Y)
{
  char sz[] = "*******";
  if (val == invalid)
  {
    sendStrXY(sz,X,Y);
  }
  else
  {
    dtostrf(val,6,4,sz);
    sendStrXY(sz,X,Y);
  }
}
*/


//NEW
static int print_coord(float val, float invalid, int X, int Y)
{
  char sz[20]; //extra headroom since dtostrf size is unclear
  char display[8];
  memset(sz,0,sizeof(sz));
  memset(display,0,sizeof(display));

  if (val == invalid)
  {
    memset(display,'*',sizeof(display)-1);
    sendStrXY(display,X,Y);
  }
  else
  {
    dtostrf(val,6,4,sz);  
    //strncpy does not auto null terminate string 
    //so memset to zero required
    strncpy(display, sz, sizeof(display)-1); 
    sendStrXY(display,X,Y);
  }
  return strlen(display);
}








//==========================================================//
// As we don't have RTC this function prints date and time obtained by GPS.
// Function recongnices actual mode and prints date and time for each one.
static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hourtmp, minute, second, hundredths;
  unsigned long age;
  char sz[17];
  char sz1[] = "**:**";
  char sz2[] = "**/**/****";
  int hour;
  
  gps.crack_datetime(&year, &month, &day, &hourtmp, &minute, &second, &hundredths, &age);
  
  hour = hourtmp;
  
  if (age == TinyGPS::GPS_INVALID_AGE) {
    if (mode == 0) {
      sprintf(sz, "%s %s", sz1, sz2);
      sendStrXY(sz,7,0);
    }
    else if (mode == 3) {
      sendStrXY("Syncing...",7,3);
    }
  }
  else
  {    
    char moment[3] = "PM";
    if (millis() - miniTimer > 500) {
      control_blink = !control_blink;
      miniTimer = millis();
    }
    hour+=TZ; // Time Zone
    
    if (hour >= 24)
    {
      ++day;
      hour -= 24;
    }
    else if (hour < 0)
    {
        --day;
        hour += 24;
    }

    if (control_clock && hour > 12) 
      hour -= 12;
    else
      moment[0] = 'A';
    
    if (control_blink)
      sprintf(sz1, "%02d:%02d", hour, minute);
    else
      sprintf(sz1, "%02d %02d", hour, minute);
        
    sprintf(sz2, "%02d/%02d/%02d", day, month, year);
    
    if (mode == 0) 
    {        
      sendStrXY(sz1, 7, 0);
      sendStrXY(sz2, 7, 6);    
    }
    else if (mode == 3)
    {
      printBigTime(sz1);
      sendStrXY(sz2, 7, 6);
      
      if(control_clock)
      {
        sendStrXY("(12h) ",7,0);
        sendCharXY(moment[0], 3, 15);
        sendCharXY(moment[1], 4, 15);
      }
      else
      {
        sendStrXY("(24h) ",7,0);
      }
    }
  }
}


//==========================================================//
// Flushes the buffer in new software serial into the TinyGPS
// object. This function is ofently call to not loose data
// (almost every char is printed on display).
static bool feedgps()
{
  while (nss.available())
  {
  char tmp = nss.read();
  if (debug)
    Serial.print(tmp);
// if (gps.encode(nss.read()))
  if (gps.encode(tmp))
      return true;
  }
  return false;
}

//==========================================================//
// Turns display on.
void displayOn(void)
{
    sendcommand(0xaf);        //display on
}

//==========================================================//
// Turns display off.
void displayOff(void)
{
  sendcommand(0xae);		//display off
}

//==========================================================//
// Clears the display by sendind 0 to all the screen map.
static void clear_display(void)
{
  unsigned char i,k;
  for(k=0;k<8;k++)
  {	
    setXY(k,0);    
    {
      for(i=0;i<128;i++)     //clear all COL
      {
        SendChar(0);         //clear all COL
        //delay(10);
      }
    }
  }
}


//==========================================================//
// Prints 'km/h' in the screen in a big font. To avoid to create 
// an entirely new font from scratch, this justs prints a static
// image.
static void printKmh_unit(void) 
{
  int X = 3;
  int Y = 9;
  setXY(X,Y);
  int salto=0; 
  for(int i=0;i<40*3;i++)     // show 40 * 24 picture
  {
    SendChar(pgm_read_byte(kmh+i));
    if(salto == 39) {
      salto = 0;
      X++;
      setXY(X,Y);
    } else {
      salto++;
    }
  }  
}


//==========================================================//
static void sendSpeed(char *string)
{

  int Y=6;
  int lon = strlen(string);
  if(lon == 3) {
    Y = 0;
  } else if (lon == 2) {
    Y = 3;
  } else if (lon == 1) {
    Y = 6;
  }
  
  int X = 2;
  while(*string)
  {
    printBigNumber(*string, X, Y);
    
    Y+=3;
    X=2;
    setXY(X,Y);
    *string++;
  }
}

/*
//==========================================================//
static void sendDistance(char *string, char *unit)
{
  int Y;
  int lon = strlen(string);
  if(lon == 3) {
    Y = 0;
  } else if (lon == 2) {
    Y = 3;
  } else if (lon == 1) {
    Y = 6;
  }
  
  int X = 2;
  while(*string)
  {
    printBigNumber(*string, X, Y);
    
    Y+=3;
    X=2;
    setXY(X,Y);
    *string++;
  }
  sendStrXY(unit, 5, 9);
}
*/


//==========================================================//
static void printBigTime(char *string)
{

  int Y;
  int lon = strlen(string);
  if(lon == 3) {
    Y = 0;
  } else if (lon == 2) {
    Y = 3;
  } else if (lon == 1) {
    Y = 6;
  }
  
  int X = 2;
  while(*string)
  {
    printBigNumber(*string, X, Y);
    
    Y+=3;
    X=2;
    setXY(X,Y);
    *string++;
  }
}

//==========================================================//
// Prints timezone in timezone setting mode.
static void print_timezone (int timezone)
{
  char sz[10];
  int X = 2;
  int Y = 6;  
  sprintf(sz, "%d", abs(timezone));
  
  // print minus simbol
  if(timezone < 0) {
    int salto=0;
    int myX = X;
    int myY = Y-3;
    for(int i=0;i<96;i++)
    {
      SendChar(pgm_read_byte(minus+i));
     
      if(salto == 23) {
        salto = 0;
        myX++;
        setXY(myX,myY);
      } else {
        salto++;
      }
    }
  }
  else
  {
    printBigNumber(' ', X, Y-3);
  }
  
  if(strlen(sz) == 2) 
    printBigNumber(sz[1], X, Y+3);
  else
    printBigNumber(' ', X, Y+3);
     
  printBigNumber(sz[0], X, Y);
}

//==========================================================//
// Prints a display big number (96 bytes) in coordinates X Y,
// being multiples of 8. This means we have 16 COLS (0-15) 
// and 8 ROWS (0-7).
static void printBigNumber(char string, int X, int Y)
{    
  setXY(X,Y);
  int salto=0;
  for(int i=0;i<96;i++)
  {
    if(string == ' ') {
      SendChar(0);
    } else 
      SendChar(pgm_read_byte(bigNumbers[string-0x30]+i));
   
    if(salto == 23) {
      salto = 0;
      X++;
      setXY(X,Y);
    } else {
      salto++;
    }
  }
}

//==========================================================//
// Actually this sends a byte, not a char to draw in the display. 
// Display's chars uses 8 byte font the small ones and 96 bytes
// for the big number font.
static void SendChar(unsigned char data) 
{
  if (interrupt && !doing_menu) return;   // Stop printing only if interrupt is call but not in button functions
  
  Wire.beginTransmission(OLED_address); // begin transmitting
  Wire.write(0x40);//data mode
  Wire.write(data);
  Wire.endTransmission();    // stop transmitting
  feedgps();    // empty serial buffer every char is printed
}

//==========================================================//
// Prints a display char (not just a byte) in coordinates X Y,
// being multiples of 8. This means we have 16 COLS (0-15) 
// and 8 ROWS (0-7).
static void sendCharXY(unsigned char data, int X, int Y)
{
  if (interrupt && !doing_menu) return; // Stop printing only if interrupt is call but not in button functions
  
  setXY(X, Y);
  Wire.beginTransmission(OLED_address); // begin transmitting
  Wire.write(0x40);//data mode
  
  for(int i=0;i<8;i++)
    Wire.write(pgm_read_byte(myFont[data-0x20]+i));
    
  Wire.endTransmission();    // stop transmitting
}

//==========================================================//
// Used to send commands to the display.
static void sendcommand(unsigned char com)
{
  Wire.beginTransmission(OLED_address);     //begin transmitting
  Wire.write(0x80);                          //command mode
  Wire.write(com);
  Wire.endTransmission();                    // stop transmitting
  feedgps();
}

//==========================================================//
// Set the cursor position in a 16 COL * 8 ROW map.
static void setXY(unsigned char row,unsigned char col)
{
  sendcommand(0xb0+row);                //set page address
  sendcommand(0x00+(8*col&0x0f));       //set low col address
  sendcommand(0x10+((8*col>>4)&0x0f));  //set high col address
}


//==========================================================//
// Prints a string regardless the cursor position.
static void sendStr(unsigned char *string)
{
  unsigned char i=0;
  while(*string)
  {
    for(i=0;i<8;i++)
    {
      SendChar(pgm_read_byte(myFont[*string-0x20]+i));
    }
    *string++;
  }
}

//==========================================================//
// Prints a string in coordinates X Y, being multiples of 8.
// This means we have 16 COLS (0-15) and 8 ROWS (0-7).
static void sendStrXY( char *string, int X, int Y)
{
  setXY(X,Y);
  unsigned char i=0;
  while(*string)
  {
    for(i=0;i<8;i++)
    {
      SendChar(pgm_read_byte(myFont[*string-0x20]+i));
    }
    *string++;
  }
}

//==========================================================//
// Print temperature
void printTemp (int X, int Y)
{
  char sz[] = "****";
  float temperature = readTemp();
   //float temp = 12.0;
  dtostrf(temperature,4,1,sz);
  sendStrXY(sz,X,Y);
  setXY(X,Y+4);
  for(int i=0;i<8;i++)     // print degree simbol
    SendChar(pgm_read_byte(myDregree+i));
  sendStrXY("C",X,Y+5);
}

//==========================================================//
// Read temperature
float readTemp()
{
  // Now take a Temerature Reading
  Wire.requestFrom(TMP75_Address,2);  // Address the TMP75 and set number of bytes to receive
  byte MostSigByte = Wire.read();              // Read the first byte this is the MSB
  byte LeastSigByte = Wire.read();             // Now Read the second byte this is the LSB

  // Being a 12 bit integer use 2's compliment for negative temperature values
  int TempSum = (((MostSigByte << 8) | LeastSigByte) >> 4); 
  // From Datasheet the TMP75 has a quantisation value of 0.0625 degreesC per bit
  float temp = (TempSum*0.0625);
  return temp;                           // Return the temperature value
}

//==========================================================//
// Inits oled and draws logo at startup
static void init_OLED(void)
{
  sendcommand(0xae);		//display off
  sendcommand(0xa6);            //Set Normal Display (default)
    // Adafruit Init sequence for 128x64 OLED module
    sendcommand(0xAE);             //DISPLAYOFF
    sendcommand(0xD5);            //SETDISPLAYCLOCKDIV
    sendcommand(0x80);            // the suggested ratio 0x80
    sendcommand(0xA8);            //SSD1306_SETMULTIPLEX
    sendcommand(0x3F);
    sendcommand(0xD3);            //SETDISPLAYOFFSET
    sendcommand(0x0);             //no offset
    sendcommand(0x40 | 0x0);      //SETSTARTLINE
    sendcommand(0x8D);            //CHARGEPUMP
    sendcommand(0x14);
    sendcommand(0x20);             //MEMORYMODE
    sendcommand(0x00);             //0x0 act like ks0108
    
    //sendcommand(0xA0 | 0x1);      //SEGREMAP   //Rotate screen 180 deg
    sendcommand(0xA0);
    
    //sendcommand(0xC8);            //COMSCANDEC  Rotate screen 180 Deg
    sendcommand(0xC0);
    
    sendcommand(0xDA);            //0xDA
    sendcommand(0x12);           //COMSCANDEC
    sendcommand(0x81);           //SETCONTRAS
    sendcommand(0xCF);           //
    sendcommand(0xd9);          //SETPRECHARGE 
    sendcommand(0xF1); 
    sendcommand(0xDB);        //SETVCOMDETECT                
    sendcommand(0x40);
    sendcommand(0xA4);        //DISPLAYALLON_RESUME        
    sendcommand(0xA6);        //NORMALDISPLAY             

  clear_display();
  sendcommand(0x2e);            // stop scroll
  //----------------------------REVERSE comments----------------------------//
  //  sendcommand(0xa0);		//seg re-map 0->127(default)
  //  sendcommand(0xa1);		//seg re-map 127->0
  //  sendcommand(0xc8);
  //  delay(1000);
  //----------------------------REVERSE comments----------------------------//
  // sendcommand(0xa7);  //Set Inverse Display  
  // sendcommand(0xae);		//display off
  sendcommand(0x20);            //Set Memory Addressing Mode
  sendcommand(0x00);            //Set Memory Addressing Mode ab Horizontal addressing mode
  //  sendcommand(0x02);         // Set Memory Addressing Mode ab Page addressing mode(RESET)  
  
   setXY(0,0);
  
  for(int i=0;i<128*8;i++)     // show 128* 64 Logo
  {
    SendChar(pgm_read_byte(logo+i));
  }
  sendcommand(0xaf);		//display on
  
  sendStrXY("by KaR]V[aN",7,5);
  delay(3000); 
}
