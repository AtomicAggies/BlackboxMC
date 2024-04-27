#include <Adafruit_GPS.h>
//MACROS
#define RX_PIN 33
#define TX_PIN 27
#define GPS_BAUD_RATE 9600

//UART COMM (Talking to transmit chip)
HardwareSerial Comm(2);

//GPS
#define GPSECHO false
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

enum AvGPSState{AV_NOFIX = 14,AV_FIX = 24} av_gps_state;

//timers
unsigned long gps_send_timer = millis();

void setup(){
  //Serial Initializations
  Serial.begin(115200);
  delay(1000);
  Serial.println( "Initializing Black Box Boot Sequence" );
  Serial.println();

  Comm.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println( "UART COMM Initialized" );

   //GPS
  av_gps_state = AV_NOFIX;
  Serial.println( "Initializing GPS" );
  init_gps();
  if(GPS.fix){
    String s = "STATE," + String(AV_FIX);
    av_gps_state = AV_FIX;
    Comm.println(s);
  }
  Serial.println( "GPS Initialized" );
  // String s = "STATE," + String(AV_FIX); //FIXME Remove line, and test if state is getting sent
  // Comm.println(s);
  Serial.println("setup");
  
}

void loop(){
  // read data from the GPS in the 'main loop'
  char c = GPS.read();

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // Serial.print(GPS.lastNMEA()); // newNMEAreceived() flag to false
    GPS.lastNMEA();
    if (!GPS.parse(GPS.lastNMEA())) //newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  if(GPS.fix && av_gps_state == AV_NOFIX && (millis()-gps_send_timer > 500)){
    String s = "STATE," + String(AV_FIX);
    av_gps_state = AV_FIX;
    Comm.println(s);
  }

}

void init_gps(){
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  if(!GPS.begin(GPS_BAUD_RATE)){
    Serial.println("Failed GPS begin");
    Comm.println("FAILED Avionics GPS begin");  // send to transmit code
  }
  else{
    Serial.println("Success GPS begin");
    Comm.println("SUCCESS Avionics GPS begin");
  }

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); //antenna status updates
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}
