#include "Adafruit_BMP3XX.h"
#include <Adafruit_GPS.h>
//MACROS
#define RX_PIN 33
#define TX_PIN 27
#define GPS_BAUD_RATE 9600
#define SEALEVELPRESSURE_HPA (1016)
#define CS 32
#define BMP_CS 14
#define BMP_SCK 5
#define BMP_MISO 19
#define BMP_MOSI 18



//UART COMM (Talking to transmit chip)
HardwareSerial Comm(2);

//GPS
#define GPSECHO false
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

enum AvGPSState{AV_NOFIX = 14,AV_FIX = 24} av_gps_state;

//BMP Stuff
Adafruit_BMP3XX bmp;
float press;
float temp;
float alt;

//timers
unsigned long gps_send_timer = millis();
unsigned long bmp_timer;
unsigned long currentTime;


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
  // if(GPS.fix){
  //   String s = "STATE," + String(AV_FIX);
  //   av_gps_state = AV_FIX;
  //   Comm.println(s);
  // }
  Serial.println( "GPS Initialized" );
  // String s = "STATE," + String(AV_FIX); //FIXME Remove line, and test if state is getting sent
  // Comm.println(s);
  Serial.println("setup");

  sd_setup();
  bmp_setup();
  bmp_timer = millis();
  
}

void loop(){
  if(GPS.fix && av_gps_state == AV_NOFIX && (millis()-gps_send_timer > 500)){
    String s = "STATE," + String(AV_FIX);
    av_gps_state = AV_FIX;
    Serial.println(s);

  currentTime = millis();
  if((currentTime - bmp_timer) > 500)
  {
    getBMP();
    //String s = "DATA," + "Temperature," + "Pressure," + "Altitude," + "ENDDATA";
    String s = "DATA," + String(temp) + "," + String(press) + "," + String(alt) + ",ENDDATA";
    Serial.println(s);
    bmp_timer = currentTime;
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

void bmp_setup() {
  while(!Serial);
  Serial.println("Adafruit BMP3XX Initiated");

  if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {
    Serial.println("No BMP detected");
    while(1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

//BMP take measurements
void bmp_test() {
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(2000);
}

void getBMP()
{
  temp = (bmp.temperature);
  press = (bmp.pressure / 100.0);
  alt = (bmp.readAltitude(SEALEVELPRESSURE_HPA));
}
