// v1.1 - first fielded version
// v1.2 - Changed SF = 10
// v1.3 - Changed for SN#2 and beyond - fixed color bitmap/
// v1.4 - Added saving last known good position, version display, etc.

#include <FlashAsEEPROM.h>

#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include <RH_RF95.h>
#include <QMC5883LCompass.h>
#include <Button.h>

// for Feather32u4 RFM9x
//#define RFM95_CS 8
//#define RFM95_RST 4
//#define RFM95_INT 7

// for feather m0 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define LCD_DISPLAY 1
//#define SERIAL_CONSOLE 1
#define COMPASS 1

#define LOCAL_GPS 1

#define SN_NO_1 0 // SN #1 has different display bitmap

const char version_str[] = {"Version: 1.4"};

// The TinyGPS++ object
TinyGPSPlus gps;
TinyGPSPlus gpsBS;   //basestation local GPS object

// last known good Remote Lat/long [v1.4]
typedef struct
{
  double Lat;
  double Long;
} GPS_Pos_struct;

GPS_Pos_struct  LKG_Position;

typedef struct {
  uint8_t x0;
  uint8_t y0;
  uint8_t x1;
  uint8_t y1;
  uint8_t x2;
  uint8_t y2;
} Tracker_struct;

#define NUM_POINTS 18
#define INCREMENT_DEGS	20

// an arrow for each 20 degs
static const Tracker_struct Tracker_icon[NUM_POINTS] = {{80,1,76,13,85,13},  //0
{102,5,92,15,102,18}, // 20
{120,16,109,22,117,29},
{135,33,121,34,127,43},
{142,53,129,50,131,60},
{142,75,131,68,129,78},
{135,96,127,85,121,94},
{120,112,117,99,109,106},
{102,123,102,110,92,113},
{80,127,85,115,75,115},
{58,123,68,113,58,110},
{40,112,51,106,43,99},
{25,96,39,94,33,85},
{18,75,31,78,29,68},
{18,53,29,60,31,50},
{25,33,33,43,39,34},
{40,16,43,29,51,22},
{58,5,58,18,68,15}};  // 340

    
int displayIndex = 0;

// These pins will also work for the 1.8" TFT shield.
#define TFT_CS        6 // Adafruit Feather
#define TFT_RST        5 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         10
#define TFT_MOSI 11  // Data out
#define TFT_SCLK 12 // Clock out

// For 1.44" and 1.8" TFT with ST7735 use:
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);   // SW SPI


#ifdef COMPASS

QMC5883LCompass compass;
#endif

// Button object = Pin 11
Button button = Button(A5,PULLUP);
// Menu definitions
#define RADIO_GPS_MENU  0
#define TRACKER_MENU  1
#define LOCAL_GPS_MENU 2

// Distance conversion
#define METERS_2_FEET 3.291

// Frequency Channels:  First channel of each subband for North America
#define RF95_FREQ_0 902.3
#define RF95_FREQ_1 903.9
#define RF95_FREQ_2 905.5
#define RF95_FREQ_3 907.1
#define RF95_FREQ_4 908.7
#define RF95_FREQ_5 910.3
#define RF95_FREQ_6 911.9
#define RF95_FREQ_7 913.5

// Flash location definition struct [v1.4]
typedef struct {
  uint8_t ChanID;
  double lkg_Lat;
  double lkg_Long;
} Flash_struct;

Flash_struct  FlashData;


float RF95_FREQ;


// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

// Battery voltage input
#define VBATPIN A7

// Last RF channel
int radio_channel;

//max altitude indicator
int max_altitude = 0;

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

#ifdef SERIAL_CONSOLE
  // Open serial communications to console
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  delay(100);
#endif

#ifdef LOCAL_GPS
  // Initial GPS Port
  Serial1.begin(9600);
#endif

#ifdef COMPASS
  //init compass
  compass.init();
  //compass.setCalibration(-336, 1190, -1160, 352, -1322, 0);   // SN #1
  compass.setCalibration(-686, 797, -1171, 267, -1237, 0);    // SN #2

#endif

#ifdef SERIAL_CONSOLE
  Serial.println("Start Listening for GPS\n");
#endif

#ifdef LCD_DISPLAY
 // Use this initializer if using a 1.8" TFT screen:
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab

#endif

  // get the data stored in flash [v1.4]
  getFlashData(&FlashData);

  LKG_Position.Lat = FlashData.lkg_Lat;
  LKG_Position.Long = FlashData.lkg_Long;

  // Provide user opportunity to change the radio channel
  radio_channel = getRadioChannel();

  Serial.print("Read Radio Chan: ");
  Serial.println(radio_channel);
  
  switch(radio_channel){
    case 0:
	    RF95_FREQ = RF95_FREQ_0;
	    break;
    case 1:
	    RF95_FREQ = RF95_FREQ_1;
	    break;
    case 2:
	    RF95_FREQ = RF95_FREQ_2;
	    break;
    case 3:
	    RF95_FREQ = RF95_FREQ_3;
	    break;
    case 4:
	    RF95_FREQ = RF95_FREQ_4;
	    break;
    case 5:
	    RF95_FREQ = RF95_FREQ_5;
	    break;
    case 6:
	    RF95_FREQ = RF95_FREQ_6;
	    break;
    case 7:
	    RF95_FREQ = RF95_FREQ_7;
	    break;
    default:
	    RF95_FREQ = RF95_FREQ_0;
	    break;
    }


  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
#ifdef SERIAL_CONSOLE
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
#endif
    while (1);
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {  
#ifdef SERIAL_CONSOLE
    Serial.println("setFrequency failed");
#endif
    while (1);
  }
#ifdef SERIAL_CONSOLE
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
#endif

 // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  //rf95.setTxPower(23, false);   // set defaults in RH_RF95.cpp

#ifdef LCD_DISPLAY
  //init display

  tft.fillScreen(ST77XX_BLACK);   // clear display

  // Display initial screen - waiting for GPS from Radio
  tft.setRotation(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(1);
  tft.setCursor(0, 30);
  tft.println(version_str); //[1.4]
  tft.println("Waiting for GPS...");
  tft.print("...on Channel ");
  tft.print(radio_channel);
#endif


  delay(2000);
 

}

uint8_t gps_len; 
uint8_t radioBuf[RH_RF95_MAX_MESSAGE_LEN];
bool first_fix = true;
int display_mode = RADIO_GPS_MENU;
int heading = 0;
bool updatedGPSFix = false;
int updateDisplayCnt = 0;

void loop() {

  ////////////////////////////////////////////
  //  check the button
  // and see if we should switch screens
  if(button.isPressed()){
     if(display_mode++ >= LOCAL_GPS_MENU){
       display_mode = RADIO_GPS_MENU;
     }

#ifdef LCD_DISPLAY
     tft.fillScreen(ST77XX_BLACK);   // clear display
#endif
     updatedGPSFix = true;  // trick later display update so that we can get a new display even if GPS doesn't update
     // Use button push to write last known position to flash
     FlashData.lkg_Lat = LKG_Position.Lat;
     FlashData.lkg_Long = LKG_Position.Long;
     UpdateFlashData(&FlashData);
  }

  //////////////////////////////////////////////
  //  Monitor the remote GPS
  // Wait for message over radio
  if (rf95.available())
  {
    // Should be a message for us now

    gps_len = sizeof(radioBuf);

    if (rf95.recv(radioBuf, &gps_len))
    {
      digitalWrite(LED, HIGH);

#ifdef SERIAL_CONSOLE
      Serial.print("Got: ");
      Serial.write((char*)radioBuf, gps_len);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

      // listen to Lora Radio and push out to the console
      printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
      printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
      printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
      printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
      printInt(gps.location.age(), gps.location.isValid(), 5);
      printDateTime(gps.date, gps.time);
      printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
    
      printInt(gps.charsProcessed(), true, 6);
      printInt(gps.sentencesWithFix(), true, 10);
      printInt(gps.failedChecksum(), true, 9);
      Serial.println();
#endif
    
      smartDelay(500, 0);

#ifdef SERIAL_CONSOLE
      if (millis() > 5000 && gps.charsProcessed() < 10)
        Serial.println(F("No GPS data received: check wiring"));
#endif

    }

    if(gps.sentencesWithFix()){
      updatedGPSFix = true;   // update loop indicator that we've had a new GPS fix
      
      // update LKG position [v1.4]
      LKG_Position.Lat = gps.location.lat();
      LKG_Position.Long = gps.location.lng();
      
      digitalWrite(LED, LOW);
    } 
  } 

#ifdef COMPASS
  //////////////////////////////////////////////////////////////////////////
  //Check compass when not waiting for message from radio
  // Read compass values
  compass.read();
  heading = compass.getAzimuth();
  // subtract mounting error of 90 degs but add the 11 deg inclination angle = -79 degs
  heading -= 79;
  if(heading < 0)
    heading = 360 + heading;

#endif

#ifdef LOCAL_GPS
  //////////////////////////////////////////////////////////////////////////
  // Then get fix from basestation GPS

  unsigned long timeout = millis() + 1000;
  uint8_t inIndex = 0;

// check if anything is in the serial buffer from the GPS
  if (Serial1.available() > 0) {
    while ( ((int32_t)(millis() - timeout) < 0) && (inIndex < (sizeof(radioBuf)/sizeof(radioBuf[0])))) {
        if (Serial1.available() > 0) {
            // read the incoming byte:
            radioBuf[inIndex] = Serial1.read();
            //Serial.write(radioBuf[inIndex]);
            if ((radioBuf[inIndex] == '\n') || (radioBuf[inIndex] == '\r')) {
                inIndex++;
                break;
            }
            inIndex++;
        }
    }
    //radioBuf[inIndex] = 0; // put a null delimiter on string
    gps_len = inIndex;
    if(!strncmp((const char *) &radioBuf[3], "GGA",3)){
#ifdef SERIAL_CONSOLE
      Serial.write(radioBuf,inIndex);
      Serial.print(" ");
      Serial.println(display_mode);
#endif
      // Now update display
      smartDelay(500, 1);
      if(gpsBS.sentencesWithFix()){
         updatedGPSFix = true;   // update loop indicator that we've had a new GPS fix
      }
    }
    inIndex = 0;
  }
#endif

  ////////////////////////////
  //  Lastly, update the display if its time

  if(((++updateDisplayCnt % 2) == 0) && updatedGPSFix)// only update display on every other instance of update GPS fix
  {
    // Check if we need to clear the screen
    if(first_fix)
    {    
#ifdef LCD_DISPLAY
      tft.fillScreen(ST77XX_BLACK);   // clear display
#endif
      first_fix = false;
    }
    //delay(500);
#ifdef LCD_DISPLAY
    updateDisplay(display_mode);  // new GPS update, update display
#endif
    updatedGPSFix = false;
  }
 
}


/* 
 *  
 *  Display the appropriate menu or display
 *  
 */
static void updateDisplay(int display)
{
  TinyGPSPlus *pGPS; 
  int distanceInMeters, courseToTarget;
  float measuredvbat;

    if(display != TRACKER_MENU)
    {
      tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
      tft.setTextSize(1);
      
      if(display == RADIO_GPS_MENU)
      {
        tft.setCursor(60, 0);
        tft.print("Remote GPS");
        pGPS = &gps;
      }
      else {
        tft.setCursor(45, 0);  
        tft.print("Basestation Data");
        pGPS = &gpsBS;
      }
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.setTextSize(2);
      tft.setCursor(0,10);
      if(display == RADIO_GPS_MENU)
      {
        tft.print(LKG_Position.Lat,6);   // print or println?
        tft.setCursor(0,30);   // how far to move cursor?
        tft.print(LKG_Position.Long,6);
        tft.setCursor(0,50);   // how far to move cursor?
        tft.print("ALT: ");
        tft.print(pGPS->altitude.meters()*METERS_2_FEET,0);
      }
      else {  
        tft.print(pGPS->location.lat(),6);   // print or println?
        tft.setCursor(0,30);   // how far to move cursor?
        tft.print(pGPS->location.lng(),6);
        tft.setCursor(0,50);   // how far to move cursor?
        tft.print("Compass: ");
        tft.print(heading);
      }
      tft.print("    ");
      tft.setCursor(0,70);   // how far to move cursor?
      tft.print("#Sats: ");
      tft.print(pGPS->satellites.value());
      tft.print(" ");
      tft.setCursor(0,90);   // how far to move cursor?
      tft.print("Time: ");
//      tft.print(pGPS->time.hour()); 
//      tft.print(":");
//      tft.print(pGPS->time.minute()); 
//      tft.print(":");
      tft.print(pGPS->time.second()); 
      
      if(display == RADIO_GPS_MENU)
      {
        tft.setCursor(0,110);   // how far to move cursor?
        tft.print("RSSI: ");
        tft.print(rf95.lastRssi(), DEC);
        tft.print("    ");
      }
	    else	// compute and display battery voltage
	    {
	      tft.setCursor(0,110);   // how far to move cursor?
        tft.print("VBat: ");
	      measuredvbat = analogRead(VBATPIN);
	      measuredvbat *= 2;    // we divided by 2, so multiply back
	      measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
	      measuredvbat /= 1024; // convert to voltage
        tft.print(measuredvbat, 2);
        tft.print("    ");
      }
    } 
    else    // If we got here, display must be tracker.  Process the tracker mode
    {
     // use last known good position from Remote GPS.  If it hasn't been updated since power-on, then it came from flash [v1.4]
     
     distanceInMeters = (int) gps.distanceBetween(gpsBS.location.lat(), gpsBS.location.lng(), LKG_Position.Lat, LKG_Position.Long);
     courseToTarget = (int) gps.courseTo(gpsBS.location.lat(), gpsBS.location.lng(),LKG_Position.Lat, LKG_Position.Long);
	
	// check for uninitialized/out of bounds distances
	if(distanceInMeters > 99999.0)
	  distanceInMeters = 99999;
      tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
      tft.setCursor(50, 40);
      tft.print("D:");
      tft.print(distanceInMeters*METERS_2_FEET, 0);  // print distance
      tft.print("    ");
      tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      tft.setCursor(50, 60);
      tft.print("A:");
      tft.print(gps.altitude.meters()*METERS_2_FEET,0);
      tft.print("    ");
      // Put update of arrows here
      // first clear the last pointer icon
     //tft.drawTriangle(Tracker_icon[displayIndex].x0, Tracker_icon[displayIndex].y0, Tracker_icon[displayIndex].x1, Tracker_icon[displayIndex].y1, Tracker_icon[displayIndex].x2, Tracker_icon[displayIndex].y2, ST77XX_BLACK);
     tft.fillTriangle(Tracker_icon[displayIndex].x0, Tracker_icon[displayIndex].y0, Tracker_icon[displayIndex].x1, Tracker_icon[displayIndex].y1, Tracker_icon[displayIndex].x2, Tracker_icon[displayIndex].y2, ST77XX_BLACK);
//     
//      tft.setRotation(1);
 //     tft.setTextSize(2);
      //tft.setCursor(60, 90);
      //tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
      // adjust heading based on compass
      courseToTarget -= heading;
      if(courseToTarget < 0)
        courseToTarget = 360 + courseToTarget;
      //tft.print(courseToTarget);
      //tft.print("  ");
      displayIndex = courseToTarget / INCREMENT_DEGS;
      if((courseToTarget % INCREMENT_DEGS) > (INCREMENT_DEGS/2))
        displayIndex++;
      if(displayIndex >= NUM_POINTS){
        displayIndex = 0;
      }
      tft.drawCircle(80, 64, 63, ST77XX_WHITE);
      //tft.drawTriangle(Tracker_icon[displayIndex].x0, Tracker_icon[displayIndex].y0, Tracker_icon[displayIndex].x1, Tracker_icon[displayIndex].y1, Tracker_icon[displayIndex].x2, Tracker_icon[displayIndex].y2, ST77XX_GREEN);
      tft.fillTriangle(Tracker_icon[displayIndex].x0, Tracker_icon[displayIndex].y0, Tracker_icon[displayIndex].x1, Tracker_icon[displayIndex].y1, Tracker_icon[displayIndex].x2, Tracker_icon[displayIndex].y2, ST77XX_GREEN);
      
    }
      
}
/**********************************************************
 * 
 * The following routines are to support the GPS processing
 * from the TinyGPS++ examples
 * 
 */
 
// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms, int local_gps)
{
  uint8_t msg_len = 0; 
  unsigned long start = millis();

  do 
  {
    while (msg_len < gps_len){
      if(local_gps)
        gpsBS.encode(radioBuf[msg_len++]);   // local GPS object
      else
        gps.encode(radioBuf[msg_len++]);   // radio GPS object
    }  
  } while (millis() - start < ms);
}

/*
 * Compute
 */

#ifdef SERIAL CONSOLE
static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0, -1);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0, -1);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0, -1);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0, -1);
}
#endif

// initial loop to allow selection of receiver channel
static int getRadioChannel(void)
{
  int TimerCnt = 10; // 10 times with 500 msec delay = 5 secs

  // First, get radio_channel from flash
  radio_channel = (int) FlashData.ChanID; // [v.14]
  if(radio_channel > 7)
    radio_channel = 0;  // only 8 channels, if we read higher, set to zero

  // Loop for 5 secs and wait for button hit.  For each button hit
  
  tft.fillScreen(ST77XX_BLACK);   // clear display
  tft.setRotation(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 30);
  tft.println("CONFIG CHANNEL: ");
  tft.print("Current Chan = ");
  tft.println(radio_channel);
  tft.println("To Change, Hit Button in 5 secs...");
  while(TimerCnt--)
  {
      if(button.isPressed()){
        tft.fillScreen(ST77XX_BLACK);   // clear display
        tft.setTextColor(ST77XX_WHITE);
        tft.setTextSize(1);
        tft.setCursor(0, 30);
        if(radio_channel++ >= 7)
          radio_channel = 0;
        tft.print("Channel = ");
        tft.println(radio_channel);
        TimerCnt = 10;
        // Write the new value into flash
        FlashData.ChanID = (uint8_t) radio_channel; // [v.14]
        UpdateFlashData(&FlashData);
        //EEPROM.write(0, (uint8_t) radio_channel);
        //EEPROM.commit();
        tft.println("Channel Updated");
      }
      delay(500);
  }
  return radio_channel;
}



static void getFlashData(Flash_struct *FlashData)
{
  uint8_t   *ptr;
  int i;
  
  if (EEPROM.isValid()) {
    // read whole struct out in serial fashion
    ptr = (uint8_t *) FlashData;
    for(i=0;i<sizeof(Flash_struct);i++)
    {
      *ptr++ = (int) EEPROM.read(i); // read each byte
    
    }
  }
  else  // if flash isn't valid, then zero everything out
  {
    FlashData->ChanID = 0;
    FlashData->lkg_Lat = 0.0;
    FlashData->lkg_Long = 0.0;
  }
}

static void UpdateFlashData(Flash_struct *FlashData)
{
  uint8_t  *ptr;
  int i;
 
  if (EEPROM.isValid()) {
    // read whole struct out in serial fashion
    ptr = (uint8_t *) &FlashData;
    for(i=0;i<sizeof(Flash_struct);i++)
    {
      EEPROM.write(i, *ptr++);
    }
    EEPROM.commit();
  }
}
