#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_GPS.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
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
#define DEBUG true

//UART COMM (Talking to transmit chip)
HardwareSerial Comm(2);

//GPS
#define GPSECHO false
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

enum AvGPSState { AV_NOFIX = 14,
                  AV_FIX = 24 } av_gps_state;

//BNO
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float pitch;
float yaw;
float roll;

  //BMP Stuff
  Adafruit_BMP3XX bmp;
float press;
float temp;
float alt;

//timers
unsigned long gps_send_timer = millis();
unsigned long bno_timer;
unsigned long bmp_timer;
unsigned long currentTime;

void init_gps() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  if (!GPS.begin(GPS_BAUD_RATE)) {
    Serial.println("Failed GPS begin");
    Comm.println("FAILED Avionics GPS begin");  // send to transmit code
  } else {
    Serial.println("Success GPS begin");
    Comm.println("SUCCESS Avionics GPS begin");
  }

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);             //antenna status updates
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}

void sd_setup() {
  Serial.println("Sensor Setup");
  // SD setup
  if (!SD.begin(CS)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  if(DEBUG)Serial.println("BLEHHHH2");
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void bmp_setup() {
  while (!Serial);

  if (!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {
    Serial.println("No BMP detected \n");
    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.println("Adafruit BMP3XX Initiated \n");
}

void bno_setup() {
  while (!Serial) delay(10);

  Serial.println("BNO Setup beginning\n");

  if (!bno.begin()) {
    Serial.println("No BNO detected, check wiring!\n");
    while (1);
  }

  delay(50);

  //Displays details and calibration values for BNO
  if(DEBUG){
    displayBNODetails();
    displayCalStatus();
    Serial.println("");
  }
  Serial.println("BNO Initiated \n");
}

//BMP take measurements
void bmp_test() {
  if (!bmp.performReading()) {
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

void getBMP() {
  temp = (bmp.temperature);
  press = (bmp.pressure / 100.0);
  alt = (bmp.readAltitude(SEALEVELPRESSURE_HPA));
}

//BNO sensor details for debug
void displayBNODetails(void) {
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" xxx");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" xxx");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayCalStatus(void) {
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system) {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("System:");
  Serial.print(system, DEC);
  Serial.print(" Gyroscope:");
  Serial.print(gyro, DEC);
  Serial.print(" Accelerometer:");
  Serial.print(accel, DEC);
  Serial.print(" Magnetometer:");
  Serial.print(mag, DEC);
}

void getBNO() {
  sensors_event_t event;
  bno.getEvent(&event);
  pitch = event.orientation.y;
  yaw = event.orientation.x;
  roll = event.orientation.z;
  //Assuming breadboard orientation (BNO above ESP)
  // X = Yaw, Y = Pitch, Z = Roll
}


// End functions


void setup() {
  //Serial Initializations
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing Black Box Boot Sequence");
  Serial.println();

  Comm.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("UART COMM Initialized");

  //GPS
  av_gps_state = AV_NOFIX;
  Serial.println("Initializing GPS");
  init_gps();
  if(GPS.fix){
    String s = "STATE," + String(AV_FIX);
    av_gps_state = AV_FIX;
    Comm.println(s);
  }
  Serial.println("GPS Initialized");

  sd_setup();
  bmp_setup();
  bno_setup();
  bno_timer = millis();
  bmp_timer = millis();

  Serial.println("Setup Fully Completed");
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
    Serial.println(s);
    Comm.println(s);
    gps_send_timer = millis();
  }

  currentTime = millis();
  if(GPS.fix && (millis()-gps_send_timer > 1000)){
    String gps_longitude = "";
    String gps_data = gps_longitude + String(GPS.longitude,4) + GPS.lon + " " + String(GPS.latitude,4) + GPS.lat;
    Serial.println(gps_data);
    Comm.println(gps_data); 
    
    gps_send_timer = millis();
  }
  

  if ((currentTime - bmp_timer) > 500) {
    getBMP();
    //String s = "DATA," + "Temperature," + "Pressure," + "Altitude," + "ENDDATA";
    String s = "DATA,BMP," + String(temp) + "," + String(press) + "," + String(alt) + ",ENDDATA";
    if(DEBUG){Serial.println(s); Serial.println("");}
    Comm.println(s);
    bmp_timer = currentTime;
  }

  if ((currentTime - bno_timer) > 500) {
    getBNO();
    String st = "DATA,BNO," + String(pitch) + "," + String(roll) + "," + String(yaw) + ",ENDDATA";
    if(DEBUG){Serial.println(st); Serial.println("");}
    Comm.println(st);
    bno_timer = currentTime;
  }
}
