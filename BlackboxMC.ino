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
#define SEALEVELPRESSURE_HPA (1013)
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
double press;
double temp;
double alt;

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

//
// SD FUNCS
//
void sd_setup() { // FIXME RETURN the SD object to be used
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
  Serial.println("SD setup complete");
  Comm.println("SD setup complete");
}
void writeFile(fs::FS &fs, const char * path, const char * message){
  Comm.printf("Writing file: %s\n", path);
  Serial.printf("Writing file: %s\n", path);
  
  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Comm.println("Failed to open file for writing");
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Comm.println("File written");
    Serial.println("File written");
  } else {
    Comm.println("Write failed");
    Serial.println("Write failed");
  }
  file.close();   
  delay(50);  // ensures file closes               
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  if(DEBUG)Comm.printf("Appending to file: %s\n", path);
  if(DEBUG)Serial.printf("Appending to file: %s\n", path);
  
  File file = fs.open(path, FILE_APPEND);
  if(!file){
     Comm.println("Failed to open file for appending");
     Serial.println("Failed to open file for appending");
     return;
  }
  if(file.print(message)){
     if(DEBUG)Comm.println("Message appended");
     if(DEBUG)Serial.println("Message appended");
  } else {
     Comm.println("Append failed");
     Serial.println("Append failed");
  }
  file.close();
  delay(50); // ensures file closes
}
// end SD FUNCS

//
// Sensor Functions
//
void bmp_setup() { 
  while (!Serial) delay(10);

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

// End Sensor functions

// arduino setup hook
void setup() {
  //Serial Initializations
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing Black Box Boot Sequence");
  Serial.println("____________________________________");

  Comm.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("UART COMM (communication line with transmit MC) Initialized");

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

  // sd_setup(); //FIXME // fix function before uncommenting
  bmp_setup();
  bno_setup();
  bno_timer = millis();
  bmp_timer = millis();

    if(!SD.begin(CS)){
        Comm.println("Card Mount Failed");
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Comm.println("No SD card attached");
        Serial.println("No SD card attached");
        return;
    }

    writeFile(SD, "/datalog.csv", "Dawson,is,the,goat\n"); //test file
    appendFile(SD, "/datalog.csv", "the,greatest,of,all,time\n");

    writeFile(SD,"/GPS.csv","GPS,Hour:Min:Sec,Latitude,Longitude,Speed,Altitude,Geoid Height,ENDDATA\n");
    writeFile(SD,"/BMP.csv","BMP,Temperature,Pressure,Altitude,ENDDATA\n");
    writeFile(SD,"/BNO.csv","BNO,Pitch,Roll,Yaw,ENDDATA\n");

  Serial.println("Setup Fully Completed");
}

// arduino loop hook
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
    String gps_ = "GPS,";
    String gps_data = gps_ + 
                      String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds) + "," + 
                      String(GPS.latitude,4) + GPS.lat + "," + 
                      String(GPS.longitude,4) + GPS.lon +  "," + 
                      String(GPS.speed) + "knots," + 
                      String(GPS.altitude) + "meters," + 
                      String(GPS.geoidheight) + "meters," + 
                      "ENDDATA\n";
    Serial.println(gps_data);
    Comm.println(gps_data); 
    appendFile(SD,"/GPS.csv",gps_data.c_str());
    
    gps_send_timer = millis();
  }
  

  if ((currentTime - bmp_timer) > 1000) {
    getBMP();
    //String s = "BMP," + "Temperature," + "Pressure," + "Altitude," + "ENDDATA";
    String s = "BMP," + String(temp) + "," + String(press) + "," + String(alt) + ",ENDDATA\n";
    if(DEBUG){Serial.println(s); Serial.println("");}
    Comm.println(s); //send to transmitMC
    appendFile(SD, "/BMP.csv", s.c_str()); // log bmp data to sd card
    bmp_timer = currentTime;
  }

  if ((currentTime - bno_timer) > 1000) {
    getBNO();
    String st = "BNO," + String(pitch) + "," + String(roll) + "," + String(yaw) + ",ENDDATA\n";
    if(DEBUG){Serial.println(st); Serial.println("");}
    Comm.println(st); //send to transmitMC
    appendFile(SD, "/BNO.csv", st.c_str()); // log bno data to sd card
    bno_timer = currentTime;
  }
}
