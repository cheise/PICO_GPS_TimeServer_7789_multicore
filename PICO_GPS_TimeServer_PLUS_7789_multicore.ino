// Network based GPS NTP server with simple WEBIF - Christian Heise <cheise@vodafone.de> - 02/06/2023
// ---RP2040 - Raspberry Pico with W5500 Network Card, DS3231 RTC and NEO6M GPS Board---
// ST7789 Version with TFT_eSPI and u8g2 font support
// Based on the work of:
// Bodmer https://github.com/Bodmer/TFT_eSPI
// Cristiano Monteiro <cristianomonteiro@gmail.com>
// Bruce E. Hall, W8BH <bhall66@gmail.com> - http://w8bh.net
// and
// https://forum.arduino.cc/u/ziggy2012/summary

#include <TFT_eSPI.h>
#include <U8g2_for_TFT_eSPI.h>

#include <SPI.h>
#include <Wire.h>  // OLED and DS3231 necessary
byte om = 99, os = 99, od =99, xsecs = 0;

// Time Server Port
#define NTP_PORT 123
static const int NTP_PACKET_SIZE = 48;
// buffers for receiving and sending data
byte packetBuffer[NTP_PACKET_SIZE];

#include <Ethernet2.h>  // Ethernet support
#include <EthernetUdp2.h> // Ethernet UDP support
#include <TimeLib.h>   // Time functions  https://github.com/PaulStoffregen/Time
#include <TinyGPSPlus.h>   // GPS parsing     https://github.com/mikalhart/TinyGPS
#include <RtcDS3231.h> // RTC functions

// Time Server MAC address
byte mac[] = {
  0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFA };

// DEBUG Mode
#define DEBUG false // Debug-Mode on/off (show Infos on Serial Port)

// Set your Static IP address
IPAddress ip(192, 168, 1, 200);       //  <--- example, put your static IP here
IPAddress subnet(255, 255, 255, 0);   // <--- example, put your subnet here
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);    // <--- example, put your Gateway IP here, when need!
IPAddress localEthernetIP;

// An Ethernet UDP instance
EthernetUDP Udp;
// An Ethernet WebServer instance for simple WEBIF
EthernetServer server(80);

#define PPS_PIN 22             // Pin on which 1PPS line is attached
#define SYNC_INTERVAL 10       // time, in seconds, between GPS sync attempts
#define SYNC_TIMEOUT 30        // time(sec) without GPS input before error
#define RTC_UPDATE_INTERVAL 30 // time(sec) between RTC SetTime events
#define PPS_BLINK_INTERVAL 50  // Set time pps led should be on for blink effect

#define rxPin 1                // Raspberry Pi Pico
#define txPin 0                // Raspberry Pi Pico
#define LOCK_LED 20            // show GPS sync
#define UNLOCK_LED 21          // show RTC sync
#define PPS_LED 25             // in this case (only Raspberry Pico or RP2040 Board's) onboard LED PI Pico, connect the external PPS_LED direct to the GPS-Board, please!

RtcDS3231<TwoWire> Rtc(Wire);  // initial RTC instance
TFT_eSPI tft = TFT_eSPI();     // initial TFT_eSPI instance, Raspberry Pico SPI_1
U8g2_for_TFT_eSPI u8f;         // initial U8g2 font instance
TinyGPSPlus gps;               // initial GPS instance

time_t displayTime = 0;        // time that is currently displayed
time_t syncTime = 0;           // time of last GPS or RTC synchronization
time_t lastSetRTC = 0;         // time that RTC was last set
volatile int pps = 0;          // GPS one-pulse-per-second flag
time_t dstStart = 0;           // start of DST in unix time
time_t dstEnd = 0;             // end of DST in unix time
bool gpsLocked = false;        // indicates recent sync with GPS
int currentYear = 0;           // used for DST

long int pps_blink_time = 0;
int resol;
int sats;
int last_resol;
int last_sats;

// --------------------------------------------------------------------------------------------------
// MAIN PROGRAM
// --------------------------------------------------------------------------------------------------

void isr() // INTERRUPT SERVICE REQUEST
{
  pps = 1;                     // Flag the 1pps input signal
  digitalWrite(PPS_LED, HIGH); // Ligth up led pps monitor
  pps_blink_time = millis();   // Capture time in order to turn led off so we can get the blink effect ever x milliseconds - On loop
}

void setup1()
{
  pinMode(LOCK_LED, OUTPUT);
  pinMode(UNLOCK_LED, OUTPUT);
  pinMode(PPS_LED, OUTPUT);
  pinMode(PPS_PIN, INPUT);

  digitalWrite(LOCK_LED, LOW);
  digitalWrite(UNLOCK_LED, HIGH);
  digitalWrite(PPS_LED, LOW);

  Serial1.setRX(rxPin);       // Raspberry Pi Pico
  Serial1.setTX(txPin);       // Raspberry Pi Pico
  Serial1.setFIFOSize(128);   // Raspberry Pi Pico
  Serial1.begin(9600);        // start GPS module UART
  delay (500);
  
  Wire.begin(); 
  Rtc.Begin();

  InitLCD(); // initialize LCD display

  Serial1.begin(9600); // set GPS baud rate to 9600 bps

  Serial.begin(115200);
  delay(500);

  // Initialize RTC
  while (!Rtc.GetIsRunning())
  {
    Rtc.SetIsRunning(true);
  }


  // never assume the Rtc was last configured by you, so
  // just clear them to your needed state
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);
  SyncWithRTC();                         // start clock with RTC data
  attachInterrupt(PPS_PIN, isr, RISING); // enable GPS 1pps interrupt input
}

void setup() // put Ethernet to one core (core0) for stability
{
  // start the Ethernet with WebServer and UDP:
  Ethernet.init(17);  // initial Ethernet Raspberry Pi Pico SPI_0
  Ethernet.begin(mac,ip);
  server.begin();
  Udp.begin(NTP_PORT);
}

void loop1()
{
  FeedGpsParser();                                    // decode incoming GPS data
  SyncCheck();                                        // synchronize to GPS or RTC
  UpdateDisplay();                                    // if time has changed, display it
  if (millis() - pps_blink_time > PPS_BLINK_INTERVAL) // If x milliseconds passed, then it's time to switch led off for blink effect
    digitalWrite(PPS_LED, LOW);
  processNTP();
  WebServer();
}

void loop(){} // Placeholder for loop, needed for RP2040 multicore

// --------------------------------------------------------------------------------------------------
// LCD SPECIFIC ROUTINES
// --------------------------------------------------------------------------------------------------

void InitLCD()
{
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  u8f.begin(tft); 
}

void UpdateDisplay()
//  Call this from the main loop
//  Updates display if time has changed
{
  time_t t = now();     // get current time
  if (t != displayTime) // has time changed?
  {
    ShowDateTime(t);    // Display the new UTC time
    ShowSyncFlag();     // show if display is in GPS sync
    displayTime = t;    // save current display value
  }
}

void ShowDate(time_t t)
{
  int d = day(t);
  int m = month(t);
  int y = year(t);

  int xpos = 45;
  int ypos = 180; // Top left corner ot clock text, about half way down
  int ysecs = ypos + 24;

  tft.setTextSize(1);
  tft.setTextColor(0x39DF, TFT_BLACK);

  if (od != d) {                                            // Redraw every day
      od = d;
      if (d < 10) xpos += tft.drawChar('0', xpos, ypos, 6); // Add day leading zero
      xpos += tft.drawNumber(d, xpos, ypos, 6);             // Draw days
      xpos += tft.drawChar('.', xpos, ypos, 6);
      if (m < 10) xpos += tft.drawChar('0', xpos, ypos, 6); // Add minutes leading zero
      xpos += tft.drawNumber(m, xpos, ypos, 6);             // Draw minutes
      xsecs = xpos;                                         // Sae seconds 'x' position for later display updates
      xpos += tft.drawChar('.', xpos, ypos, 6);
      tft.drawNumber(y, xpos, ypos, 6);
    }

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  u8f.setFont(u8g2_font_open_iconic_all_4x_t);
  u8f.setForegroundColor(0xA3A0);
  u8f.drawGlyph(0, 215, 107);
  u8f.setForegroundColor(TFT_WHITE);
}

void ShowTime(time_t t)
{
  int h = hour(t);
  int m = minute(t);
  int s = second(t);

  int xpos = 45;
  int ypos = 110; // Top left corner ot clock text, about half way down
  int ysecs = ypos + 24;

  tft.drawLine(0, 90, 320, 90, TFT_WHITE);
  tft.setTextSize(1);
  tft.setTextColor(0x6500, TFT_BLACK);
  
    if (om != m) {                                          // Redraw hours and minutes time every minute
      om = m;
      if (h < 10) xpos += tft.drawChar('0', xpos, ypos, 6); // Add hours leading zero for 24 hr clock
      xpos += tft.drawNumber(h, xpos, ypos, 6);             // Draw hours
      xpos += tft.drawChar(':', xpos, ypos, 6);
      if (m < 10) xpos += tft.drawChar('0', xpos, ypos, 6); // Add minutes leading zero
      xpos += tft.drawNumber(m, xpos, ypos, 6);             // Draw minutes
      xsecs = xpos;                                         // Sae seconds 'x' position for later display updates
    }
    if (os != s) { // Redraw seconds time every second
      os = s;
      xpos = xsecs;
      xpos += tft.drawChar(':', xsecs, ypos, 6); // Seconds colon
    }
    //Draw seconds
    if (s < 10) xpos += tft.drawChar('0', xpos, ypos, 6); // Add leading zero
    tft.drawNumber(s, xpos, ypos, 6);
    tft.drawLine(0, 165, 320, 165, TFT_WHITE); 

  u8f.setFont(u8g2_font_open_iconic_all_4x_t);
  u8f.setFontMode(1);
  u8f.setForegroundColor(TFT_WHITE);
  u8f.setBackgroundColor(TFT_BLACK);

  u8f.setForegroundColor(TFT_MAGENTA);
  u8f.drawGlyph(0, 145, 123);

  u8f.setFont(u8g2_font_helvB18_tf);
  u8f.drawStr(250, 142, "UTC");
  u8f.setForegroundColor(TFT_WHITE);
}

void ShowDateTime(time_t t)
{
  ShowDate(t);

  ShowTime(t);
}

void ShowSyncFlag()
{
  if (gps.satellites.value() != 255) {
    sats = gps.satellites.value();
  }
  else {
    sats = 1;
  }
  
  if (gpsLocked) {
    digitalWrite(LOCK_LED, HIGH), digitalWrite(UNLOCK_LED, LOW);
  }
  else {
    digitalWrite(LOCK_LED, LOW), digitalWrite(UNLOCK_LED, HIGH);
  }

  if (gpsLocked) {
    resol = gps.hdop.value() / 100;  // unit 1/100
  }
  else {
    resol = 0;
  }
  
  u8f.setFontMode(1);
  u8f.setForegroundColor(TFT_WHITE);
  u8f.setBackgroundColor(TFT_BLACK);
  u8f.setFont(u8g2_font_open_iconic_all_4x_t);
  u8f.setForegroundColor(TFT_BLUE);
  u8f.drawGlyph(0, 35, 259);
  u8f.drawGlyph(170, 35, 263);
  u8f.setForegroundColor(TFT_WHITE);
  
  u8f.setFont(u8g2_font_helvB18_tf);
  u8f.setForegroundColor(TFT_YELLOW);
  u8f.drawStr(45, 28, "SAT");
  u8f.drawStr(215, 28, "RESOL");
  u8f.setForegroundColor(TFT_WHITE);
  tft.drawLine(160, 0, 160, 90, TFT_WHITE);

  int xpos_resol = 240;
  int ypos_resol = 40;
  
  tft.setTextColor(TFT_BLACK, TFT_BLACK);
  if (resol != last_resol) tft.drawNumber(last_resol, xpos_resol, ypos_resol, 6);
  if (resol <= 2) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
  }
  else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
  }
  tft.drawNumber(resol, xpos_resol, ypos_resol, 6);
  last_resol = resol;
 

  int xpos_sats = 45;
  int ypos_sats = 40;
  
  tft.setTextColor(TFT_BLACK, TFT_BLACK);
  if (sats != last_sats) tft.drawNumber(last_sats, xpos_sats, ypos_sats, 6);
  if (sats >=5) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
  }
  else {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  }
  if (sats <= 9) xpos_sats = xpos_sats + 26;
  tft.drawNumber(sats, xpos_sats, ypos_sats, 6);
  last_sats = sats;
}

// --------------------------------------------------------------------------------------------------
//  RTC SUPPORT
// --------------------------------------------------------------------------------------------------

// Update RTC from current system time
void SetRTC(time_t t)
{
  RtcDateTime timeToSet;
  timeToSet.InitWithUnix32Time(t);
  Rtc.SetDateTime(timeToSet);
}

void UpdateRTC()
// keep the RTC time updated by setting it every (RTC_UPDATE_INTERVAL) seconds
// should only be called when system time is known to be good, such as in a GPS sync event
{
  time_t t = now();                            // get current time
  if ((t - lastSetRTC) >= RTC_UPDATE_INTERVAL) // is it time to update RTC internal clock?
  {
    SetRTC(t);      // set RTC with current time
    lastSetRTC = t; // remember time of this event
    #if DEBUG
    Serial.println("Update RTC...");
    #endif  
  }
}

// --------------------------------------------------------------------------------------------------
// TIME SYNCHONIZATION ROUTINES
// --------------------------------------------------------------------------------------------------

void SyncWithGPS()
{
  int y;
  byte h, m, s, mon, d, hundredths;
  unsigned long age;
  //gps.crack_datetime(&y, &mon, &d, &h, &m, &s, hundredths, &age); // get time from GPS
        
  y = (gps.date.year());
  mon = (gps.date.month());
  d = (gps.date.day());
 
  h = (gps.time.hour());
  m = (gps.time.minute());
  s = (gps.time.second());
  hundredths = (gps.time.centisecond());
  age = (gps.time.age());
  
  
  if (age < 1000)                             // dont use data older than 1 second
  {
    setTime(h, m, s, d, mon, y); // copy GPS time to system time
    adjustTime(1);                     // 1pps signal = start of next second
    syncTime = now();                  // remember time of this sync
    #if DEBUG
    Serial.print("SyncFromGPS: ");
    Serial.println(syncTime);
    Serial.print("Age: ");
    Serial.println((age));
    #endif
    gpsLocked = true;                  // set flag that time is reflects GPS time
    UpdateRTC();                       // update internal RTC clock periodically
  }
}

void SyncWithRTC()
{
  RtcDateTime time = Rtc.GetDateTime();
  long int a = time.Unix32Time();
  setTime(a); // set system time from RTC
  #if DEBUG
  Serial.println("SyncFromRTC: ");
  Serial.println(a);
  #endif
  syncTime = now();                       // and remember time of this sync event
}

void SyncCheck()
// Manage synchonization of clock to GPS module
// First, check to see if it is time to synchonize
// Do time synchonization on the 1pps signal
// This call must be made frequently (keep in main loop)
{
  unsigned long timeSinceSync = now() - syncTime; // how long has it been since last sync?
  if (pps && (timeSinceSync >= SYNC_INTERVAL))
  { // is it time to sync with GPS yet?
    SyncWithGPS(); // yes, so attempt it.
  }
  pps = 0;                           // reset 1-pulse-per-second flag, regardless
  if (timeSinceSync >= SYNC_TIMEOUT) // GPS sync has failed
  {
    gpsLocked = false; // flag that clock is no longer in GPS sync
    SyncWithRTC(); // sync with RTC instead
  }
}

void FeedGpsParser()
// feed currently available data from GPS module into tinyGPS parser
{
  while (Serial1.available()) // look for data from GPS module
  {
    char c = Serial1.read(); // read in all available chars
    gps.encode(c);      // and feed chars to GPS parser
    //Serial.write(c); // Uncomment for some extra debug info if in doubt about GPS feed
  }
}

void processNTP()
{

  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    IPAddress Remote = Udp.remoteIP();
    int PortNum = Udp.remotePort();

#if DEBUG
    Serial.println();
    Serial.print("Received UDP packet size ");
    Serial.println(packetSize);
    Serial.print("From ");

    for (int i = 0; i < 4; i++)
    {
      Serial.print(Remote[i], DEC);
      if (i < 3)
      {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.print(PortNum);

    byte LIVNMODE = packetBuffer[0];
    Serial.print("  LI, Vers, Mode :");
    Serial.print(packetBuffer[0], HEX);

    byte STRATUM = packetBuffer[1];
    Serial.print("  Stratum :");
    Serial.print(packetBuffer[1], HEX);

    byte POLLING = packetBuffer[2];
    Serial.print("  Polling :");
    Serial.print(packetBuffer[2], HEX);

    byte PRECISION = packetBuffer[3];
    Serial.print("  Precision :");
    Serial.println(packetBuffer[3], HEX);

    for (int z = 0; z < NTP_PACKET_SIZE; z++)
    {
      Serial.print(packetBuffer[z], HEX);
      if (((z + 1) % 4) == 0)
      {
        Serial.println();
      }
    }
    Serial.println();

#endif

    packetBuffer[0] = 0b00100100; // LI, Version, Mode
    //packetBuffer[1] = 1 ;   // stratum
    //think that should be at least 4 or so as you do not use fractional seconds

    packetBuffer[1] = 4;    // stratum
    packetBuffer[2] = 6;    // polling minimum
    packetBuffer[3] = 0xFA; // precision

    packetBuffer[7] = 0; // root delay
    packetBuffer[8] = 0;
    packetBuffer[9] = 8;
    packetBuffer[10] = 0;

    packetBuffer[11] = 0; // root dispersion
    packetBuffer[12] = 0;
    packetBuffer[13] = 0xC;
    packetBuffer[14] = 0;

    //int year;
    //byte month, day, hour, minute, second, hundredths;
    unsigned long date, time, age;
    uint32_t timestamp, tempval;
    time_t t = now();

    timestamp = now() + 2208988800UL;

#if DEBUG
    Serial.println(timestamp);
#endif

    tempval = timestamp;

    packetBuffer[12] = 71; //"G";
    packetBuffer[13] = 80; //"P";
    packetBuffer[14] = 83; //"S";
    packetBuffer[15] = 0;  //"0";

    // reference timestamp
    packetBuffer[16] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[17] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[18] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[19] = (tempval)&0xFF;

    packetBuffer[20] = 0;
    packetBuffer[21] = 0;
    packetBuffer[22] = 0;
    packetBuffer[23] = 0;

    //copy originate timestamp from incoming UDP transmit timestamp
    packetBuffer[24] = packetBuffer[40];
    packetBuffer[25] = packetBuffer[41];
    packetBuffer[26] = packetBuffer[42];
    packetBuffer[27] = packetBuffer[43];
    packetBuffer[28] = packetBuffer[44];
    packetBuffer[29] = packetBuffer[45];
    packetBuffer[30] = packetBuffer[46];
    packetBuffer[31] = packetBuffer[47];

    //receive timestamp
    packetBuffer[32] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[33] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[34] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[35] = (tempval)&0xFF;

    packetBuffer[36] = 0;
    packetBuffer[37] = 0;
    packetBuffer[38] = 0;
    packetBuffer[39] = 0;

    //transmitt timestamp
    packetBuffer[40] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[41] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[42] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[43] = (tempval)&0xFF;

    packetBuffer[44] = 0;
    packetBuffer[45] = 0;
    packetBuffer[46] = 0;
    packetBuffer[47] = 0;

    // Reply to the IP address and port that sent the NTP request

    Udp.beginPacket(Remote, PortNum);
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
  }
}

// --------------------------------------------------------------------------------------------------
// SIMPLE WEBSERVER ROUTINES
// --------------------------------------------------------------------------------------------------

void WebServer () 
{
  EthernetClient client = server.available();
  if (client) {
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    localEthernetIP = Ethernet.localIP();
    while (client.connected()) {
      if (client.available()) {
        char d = client.read();
        if (d == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          client.println("<head>");
          client.println("<title>GPS-NTP Server</title>");
          client.println("<body bgcolor=#000000>");
          client.println("<meta http-equiv='refresh' content='1'>");
          client.println("</head>");
          client.println("<h1><center><font color=0000ff><u>GPS-NTP Server with Realtime-Clock is online</u></font></center></h1>");
          client.println("<br>");
          client.print("<h2><center><font color=ffffff>IP-Address: ");
          client.print(localEthernetIP);
          client.print("</font></h2></center>");
          if (gpsLocked) {
            client.print("<h2><center><b><font color=00ff00>GPS locked with: ");
            client.print(gps.satellites.value());
            client.print(" Sattelites :)</font></b></center></h2>");
          }
          else {
            client.print("<h2><center><b><font color=ff0000>GPS unlocked with 0 Sattelites :(</font></b></center></h2>");
          }
          client.println("</html>");
          break;
        }
        if (d == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } 
        else if (d != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
  }
}
