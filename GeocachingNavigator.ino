// Test code for Adafruit Flora GPS modules
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Flora GPS module
// ------> http://adafruit.com/products/1059
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada
#include <Adafruit_GPS.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <Time.h>
#include <Wire.h>
#include <LSM303.h>

Adafruit_GPS GPS(&Serial1);
LSM303 compass;

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;

// Number of targets
int NTargets = 5;

// Number of LEDs in Ring
int NLEDs = 16;

// The first number of this matrix has to be the number of targets, the second number is always 2
// The matrix gives the decimal WGS gps coordinates of each destination
// DON'T FORGET THE COMMA BETWEEN THE TWO COORDINATES
float TargetLatLon[18][2] PROGMEM = {
  {33.76907, -84.37565}, // GC Central Park
  {33.77222, -84.37845}, // GC Krispy (Atl)
  {33.77545, -84.37518}, // GC Cruzin to a Cache
  {33.77243, -84.38477}, // GC Rock the Casbah    
  {30.75465, -81.65}, // On My Mind (Southern Georgia)
  //// WINDSOR CACHES
  {42.32186, -82.90516}, // GC Just for Nicholas (Windsor)
  {42.308133, -82.987233},
  {42.313400, -82.9863},
  {42.312066, -82.9931},
  {42.320867, -82.92505},
  {42.324033, -82.921667},
  {42.323533, -82.92445},
  {42.324533, -82.926083},
  {42.319017, -82.92375},
  {42.321983, -82.930217},
  {42.323533, -82.92445},
  {42.309533, -82.983833},
};  

// Threshold distance for stuff *****
float tripSegment = 1000;    // units are meters 

// integer arrays for colors
int RedsArray[16];
int GrnsArray[16];
int BlusArray[16];

//--------------------------------------------------|
//Your NeoPixel ring may not line up with ours. |
//Enter which NeoPixel led is your top LED (0-15). |
#define TOP_LED 14
//--------------------------------------------------|
//Your compass module may not line up with ours. |
//Once you run compass mode, compare to a separate |
//compass (like one found on your smartphone). |
//Point your TOP_LED north, then count clockwise |
//how many LEDs away from TOP_LED the lit LED is |
#define LED_OFFSET 0
//--------------------------------------------------|
// Flag that indicates if there is a destination in range
int TargetNearby = 0;

// Trip distance
float tripDistance;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, 6, NEO_GRB + NEO_KHZ800);

// Offset hours from gps time (UTC)
//const int offset = 1; // Central European Time
const int offset = -4; // Eastern Daylight Time (USA)
//const int offset = -5; // Central Daylight Time (USA)
//const int offset = -8; // Pacific Standard Time (USA)
//const int offset = -7; // Pacific Daylight Time (USA)

int topLED = TOP_LED;
int compassOffset = LED_OFFSET;

//int lastMin = 16;
//int lastHour = 16;
int startLED = 0;
int startLEDlast = 16;
int lastCombined = 0;
int startGPS = 0;
int mode = 0;
int lastDir = 16;
int dirLED_r = 0;
int dirLED_g = 0;
int dirLED_b = 255;
int compassReading;

// Pushbutton setup
int buttonPin = 10; // the number of the pushbutton pin
int buttonState; // the current reading from the input pin
int lastButtonState = HIGH; // the previous reading from the input pin
long buttonHoldTime = 0; // the last time the output pin was toggled
long buttonHoldDelay = 1500; // how long to hold the button down

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0; // the last time the output pin was toggled
long debounceDelay = 50; // the debounce time; increase if the output flickers
long menuDelay = 2500;
long menuTime;

float fLat = 0.0;
float fLon = 0.0;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup()
{
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  compass.init();
  compass.enableDefault();

  // Calibration values. Use the Calibrate example program to get the values for your compass.
  compass.m_min.x = -809; 
  compass.m_min.y = -491; 
  compass.m_min.z = -793; //   why are these values never used again?
  compass.m_max.x = +451; 
  compass.m_max.y = +697; 
  compass.m_max.z = 438;  //   why are these values never used again?

  delay(1000);
  // Ask for firmware version
//  Serial1.println(PMTK_Q_RELEASE);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Make input & enable pull-up resistors on switch pins for pushbutton
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, HIGH);
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~     
uint32_t gpsTimer = millis();
uint32_t startupTimer = millis();
uint32_t compassTimer = millis();
//// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void loop() // run over and over again
{
  compassCheck();
  // read the state of the switch into a local variable:
  int buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) {
    buttonCheck();
  }

  lastButtonState = buttonState;

  //Serial.println(buttonState);
  // read data from the GPS in the 'main loop'
   char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
  if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    return; // we can fail to parse a sentence in which case we should just wait for another
  }

    Serial.print("Fix: "); Serial.println((int)GPS.fix);
    Serial.println("-- ");  
//
//  // if millis() or timer wraps around, we'll just reset it
//  if (gpsTimer > millis()) gpsTimer = millis();
//
  if (startGPS == 0) {
    if (GPS.fix) {
      // set the Time to the latest GPS reading
//      setTime(GPS.hour, GPS.minute, GPS.seconds, GPS.day, GPS.month, GPS.year);
//      delay(50);
//      adjustTime(offset * SECS_PER_HOUR);
//      delay(500);

      startGPS = 1;
    }
  }
//  // approximately every 60 seconds or so, update time
//  if ((millis() - gpsTimer > 60000) && (startGPS == 1)) {
//    gpsTimer = millis(); // reset the timer
//    if (GPS.fix) {
//      // set the Time to the latest GPS reading
//      setTime(GPS.hour, GPS.minute, GPS.seconds, GPS.day, GPS.month, GPS.year);
//      delay(50);
//      adjustTime(offset * SECS_PER_HOUR);
//      delay(500);
//    }
//  }
//
  // OBTAIN LAT AND LONG OF CURRENT LOCATION IN DEGREES
  // fLat and fLon are global variables
  if (GPS.fix) {
    fLat = decimalDegrees(GPS.latitude, GPS.lat);
    fLon = decimalDegrees(GPS.longitude, GPS.lon);
  }

  if (mode == 0) {
//    navMode();
    compassMode();
  }
  if (mode == 1) {
    navMode();
  }
}
//// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Fill pixels in one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {  
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}
//// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void buttonCheck() {
  menuTime = millis(); // I THINK WE CAN REMOVE THIS LINE
  int buttonState = digitalRead(buttonPin);
  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonHoldTime = millis();
  }
  if (buttonState == LOW && lastButtonState == LOW) {
    if ((millis() - buttonHoldTime) > buttonHoldDelay) {
      if(mode == 2) {  // WIPE EVERYTHING ON 3 (DO I NEED THE TURNOFF FUNCTION AND MODE?)
        mode = 0;
        //        lastMin = 16;
        //        lastHour = 16;
        colorWipe(strip.Color(0, 0, 0), 20);
        buttonHoldTime = millis();
      } 
      else {
        mode = mode + 1;
        colorWipe(strip.Color(0, 0, 0), 20);
        buttonHoldTime = millis();
      }
    }
  }
}
//// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
//// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//// NAVIGATION MODE
void navMode() {
  Serial.println("Navigation Mode Activated. GPS Fix and startGPS:");
  Serial.println(GPS.fix);
  Serial.println(startGPS);
  
  if ((startGPS == 1) && (GPS.fix)) {

    //    Serial.println("Signal Found");
//    strip.setPixelColor(0, strip.Color(255, 0, 0));
//    strip.show();

    float TargetLat;
    float TargetLon;
    float TempDist = 0.0;
    int DN;

    // This must obtain the current orientation of device
    compassCheck();

    // ADD LOOP OVER ALL DESTINATIONS
    for (int i=0; i < NTargets; i++) {

      // READ TARGET LAT/LONG
      TargetLat = pgm_read_float(&TargetLatLon[i][0]); // use pgm_read_float to convert from array, not sure why
      TargetLon = pgm_read_float(&TargetLatLon[i][1]);    

      // Get a temp distance to target i
      TempDist = (double)calc_dist(fLat, fLon, TargetLat, TargetLon);

      // if distance is less or equal to threshold, then compute colors, store in color vectors
      if (TempDist <= tripSegment) {
        
        TargetNearby = 1;            
        
        // Compute RBG colors
        headingDistanceLinear(TempDist);

        // Find direction number
        if ((calc_bearing(fLat, fLon, TargetLat, TargetLon) - compassReading) > 0) {
          DN = (int)compassDirection(calc_bearing(fLat, fLon, TargetLat, TargetLon)-compassReading);
        } 
        else {
          DN = (int)compassDirection(calc_bearing(fLat, fLon, TargetLat, TargetLon)-compassReading+360);
        }   
        
//        Serial.println("----- ");        
//        Serial.println("Red, DN, TempDist, i: ");
//        Serial.print(dirLED_r);Serial.println(" ");Serial.print(DN);Serial.println(" ");
//        Serial.print(TempDist);Serial.println(" ");Serial.print(i);
//        Serial.println("~ ~ ~ ~ ~ ~ ~");                
        
        // At each given pixel, we want to only show the closest target
        // Display a Target at a pixel if it is closer than other targets at that pixel
        // Don't forget: the closer the target, the more "red" it is
        // Don't forget: the closer the target, the more "green" it is
        if ((TempDist < tripSegment/2) && (RedsArray[DN] < dirLED_r)) {
          RedsArray[DN] = dirLED_r;
          GrnsArray[DN] = dirLED_g;
          BlusArray[DN] = dirLED_b;    

        }
        if ((TempDist < tripSegment) && (TempDist >= tripSegment/2) && (dirLED_g < GrnsArray[DN])) {
          RedsArray[DN] = dirLED_r;
          GrnsArray[DN] = dirLED_g;
          BlusArray[DN] = dirLED_b;     
        }
      };
    }

    // Check if any destinations are within threshold
    if (TargetNearby == 1) {
      // Assign Colours to Each LED in Ring 
      for (int i=0; i < NLEDs; i++) {
        strip.setPixelColor(i, strip.Color(RedsArray[i],GrnsArray[i],BlusArray[i]));
      }
      strip.show();      // Show LED Settings
      delay(1000); 
    }
    else if (TargetNearby == 0) { // display circles in yellow if all distances are far away
      strip.setPixelColor(startLEDlast, strip.Color(0, 0, 0));
      strip.setPixelColor(startLED, strip.Color(64, 96, 0));
      strip.show();
      startLEDlast = startLED;
      startLED++;
      startLED = startLED % (NLEDs);
    }

    // Reset Colour Arrays 
    for (int N=0; N < NLEDs; N++) {
      RedsArray[N]=0;GrnsArray[N]=0;BlusArray[N]=0;
    }    
    TargetNearby = 0;
    delay(20);   
  }
  else {
    // if millis() or timer wraps around, we'll just reset it
    if (startupTimer > millis()) startupTimer = millis();

    // approximately every 10 seconds or so, update time
    if (millis() - startupTimer > 200) {
      startupTimer = millis(); // reset the timer
      if (startLED == 16) {
        startLED = 0;
      }
      // THIS LOOKS LIKE CIRCLES IN BLUE
      strip.setPixelColor(startLEDlast, strip.Color(0, 0, 0));
      strip.setPixelColor(startLED, strip.Color(0, 0, 255));
      strip.show();
      startLEDlast = startLED;
      startLED++;
    }
  }
//  delay(1000);  // Adding a delay here created a lot of problems. Leave this commented out. 
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int calc_bearing(float flat1, float flon1, float flat2, float flon2)
// This code only relies on GPS data, not compass data
// a useful websites:
//   http://www.movable-type.co.uk/scripts/latlong.html
//   http://mathforum.org/library/drmath/view/55417.html
// The equations that are used here don't make much sense to me. Are they using a Taylor approximation for sine? Where does the 57.3 and 69.1 numbers come from?
{
  float calc;
  float bear_calc;

  float x = 69.1 * (flat2 - flat1);
  float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);

  calc=atan2(y,x);

  bear_calc= degrees(calc);

  if(bear_calc<=1){
    bear_calc=360+bear_calc;
  }
  return bear_calc;
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void headingDistanceLinear(int fDist)
{
  //Use this part of the code to determine how far you are away from the destination.

  float tripSegment2 = tripSegment/2;
  float brightness = 0.75; // a number between 0 and 1 that controls the brightness of the LEDs
  if (fDist >= (tripSegment)) { // You are more than tripSegment meters from your destination
    dirLED_r = 0;
    dirLED_g = 0;
    dirLED_b = 0;
  }
  if ((fDist >= tripSegment2) && (fDist < tripSegment)) { // You are between tripSegment/2 and tripSegment meters from your destination
    dirLED_r = 0; 
    dirLED_g = brightness*255*(tripSegment - fDist)/tripSegment2; // add more green as the dist becomes smaller
    dirLED_b = brightness*255*(fDist-tripSegment2)/tripSegment2;  // add less blue as the dist becomes smaller
  }
  if ((fDist >= 0)&&(fDist < tripSegment2)) {// You are now within tripSegment/2 meters of your destination
    dirLED_r = brightness*255*(1-fDist/tripSegment2);
    dirLED_g = brightness*255*((fDist)/tripSegment2);
    dirLED_b = 0;
  }
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

unsigned long calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  diflat=radians(flat2-flat1);
  flat1=radians(flat1);
  flat2=radians(flat2);
  diflon=radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

  dist_calc*=6371000.0; //Converting to meters

  return dist_calc;
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Convert NMEA coordinate to decimal degrees
float decimalDegrees(float nmeaCoord, char dir) {
  uint16_t wholeDegrees = 0.01*nmeaCoord;
  int modifier = 1;

  if (dir == 'W' || dir == 'S') {
    modifier = -1;
  }

  return (wholeDegrees + (nmeaCoord - 100.0*wholeDegrees)/60.0) * modifier;
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void compassMode() {
  dirLED_r = 0;
  dirLED_g = 0;
  dirLED_b = 255;
  compassDirection(compassReading);
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 
void compassCheck() {
  // if millis() or timer wraps around, we'll just reset it
  if (compassTimer > millis()) compassTimer = millis();

  // approximately every 10 seconds or so, update time
  if (millis() - compassTimer > 50) {
    compassTimer = millis(); // reset the timer
    compass.read();
    compassReading = compass.heading((LSM303::vector<int>){
      0,+1,0    }
    ); // 
  }
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  
unsigned int compassDirection(int compassHeading)
{
  Serial.print("Compass Direction: ");
  Serial.println(compassHeading);

  unsigned int ledDir = 2;
  int tempDir = 0;
  //Use this part of the code to determine which way you need to go.
  //Remember: this is not the direction you are heading, it is the direction to the destination (north = forward).
  if ((compassHeading > 348.75)||(compassHeading < 11.25)) {
    //Serial.println(" N");
    //Serial.println("Forward");
    if (mode == 1 ) {
      tempDir = topLED;
    } 
    else {
      tempDir = topLED;
    }
  }

  if ((compassHeading >= 11.25)&&(compassHeading < 33.75)) {
    //Serial.println("NNE");
    //Serial.println("Go Right");
    if (mode == 1 ) {
      tempDir = topLED - 1;
    } 
    else {
      tempDir = topLED + 1;
    }
  }

  if ((compassHeading >= 33.75)&&(compassHeading < 56.25)) {
    //Serial.println(" NE");
    //Serial.println("Go Right");
    if (mode == 1 ) {
      tempDir = topLED - 2;
    } 
    else {
      tempDir = topLED + 2;
    }
  }

  if ((compassHeading >= 56.25)&&(compassHeading < 78.75)) {
    //Serial.println("ENE");
    //Serial.println("Go Right");
    if (mode == 1 ) {
      tempDir = topLED - 3;
    } 
    else {
      tempDir = topLED + 3;
    }
  }

  if ((compassHeading >= 78.75)&&(compassHeading < 101.25)) {
    //Serial.println(" E");
    //Serial.println("Go Right");
    if (mode == 1 ) {
      tempDir = topLED - 4;
    } 
    else {
      tempDir = topLED + 4;
    }
  }

  if ((compassHeading >= 101.25)&&(compassHeading < 123.75)) {
    //Serial.println("ESE");
    //Serial.println("Go Right");
    if (mode == 1 ) {
      tempDir = topLED - 5;
    } 
    else {
      tempDir = topLED + 5;
    }
  }

  if ((compassHeading >= 123.75)&&(compassHeading < 146.25)) {
    //Serial.println(" SE");
    //Serial.println("Go Right");
    if (mode == 1 ) {
      tempDir = topLED - 6;
    } 
    else {
      tempDir = topLED + 6;
    }
  }

  if ((compassHeading >= 146.25)&&(compassHeading < 168.75)) {
    //Serial.println("SSE");
    //Serial.println("Go Right");
    if (mode == 1 ) {
      tempDir = topLED - 7;
    } 
    else {
      tempDir = topLED + 7;
    }
  }

  if ((compassHeading >= 168.75)&&(compassHeading < 191.25)) {
    //Serial.println(" S");
    //Serial.println("Turn Around");
    if (mode == 1 ) {
      tempDir = topLED - 8;
    } 
    else {
      tempDir = topLED + 8;
    }
  }

  if ((compassHeading >= 191.25)&&(compassHeading < 213.75)) {
    //Serial.println("SSW");
    //Serial.println("Go Left");
    if (mode == 1 ) {
      tempDir = topLED - 9;
    } 
    else {
      tempDir = topLED + 9;
    }
  }

  if ((compassHeading >= 213.75)&&(compassHeading < 236.25)) {
    //Serial.println(" SW");
    //Serial.println("Go Left");
    if (mode == 1 ) {
      tempDir = topLED - 10;
    } 
    else {
      tempDir = topLED + 10;
    }
  }

  if ((compassHeading >= 236.25)&&(compassHeading < 258.75)) {
    //Serial.println("WSW");
    //Serial.println("Go Left");
    if (mode == 1 ) {
      tempDir = topLED - 11;
    } 
    else {
      tempDir = topLED + 11;
    }
  }

  if ((compassHeading >= 258.75)&&(compassHeading < 281.25)) {
    //Serial.println(" W");
    //Serial.println("Go Left");
    if (mode == 1 ) {
      tempDir = topLED - 12;
    } 
    else {
      tempDir = topLED + 12;
    }
  }

  if ((compassHeading >= 281.25)&&(compassHeading < 303.75)) {
    //Serial.println("WNW");
    //Serial.println("Go Left");
    if (mode == 1 ) {
      tempDir = topLED - 13;
    } 
    else {
      tempDir = topLED + 13;
    }
  }

  if ((compassHeading >= 303.75)&&(compassHeading < 326.25)) {
    //Serial.println(" NW");
    //Serial.println("Go Left");
    if (mode == 1 ) {
      tempDir = topLED - 14;
    } 
    else {
      tempDir = topLED + 14;
    }
  }

  if ((compassHeading >= 326.25)&&(compassHeading < 348.75)) {
    //Serial.println("NWN");
    //Serial.println("Go Left");
    if (mode == 1 ) {
      tempDir = topLED - 15;
    } 
    else {
      tempDir = topLED + 15;
    }
  }

  if (tempDir > 15) {
    ledDir = tempDir - 16;
  } 
  else {
    ledDir = tempDir;
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // SET LEDDIR BASED ON TEMPDIR
  if (tempDir < 0) {
    ledDir = tempDir + 16;
  } 
  else {
    ledDir = tempDir;
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  
  if (mode == 1) {
    ledDir = ledDir + compassOffset;
    if (ledDir > 15) {
      ledDir = ledDir - 16;
    }
  } 
  else {
    ledDir = ledDir + compassOffset;
    if (ledDir > 15) {
      ledDir = ledDir - 16;
    }
  }

  if ((lastDir != ledDir) && (mode == 0)) {
    strip.setPixelColor(lastDir, strip.Color(0, 0, 0));
    strip.setPixelColor(ledDir, strip.Color(dirLED_r, dirLED_g, dirLED_b));
    strip.show();
    lastDir = ledDir;
  }
  return ledDir;
}


