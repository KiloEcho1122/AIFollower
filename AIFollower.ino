#include <HMC5883L.h>




// Imports
#include <Wire.h>
#include <SoftwareSerial.h>  
#include "./TinyGPS.h"                 // Use local version of this library
#include "./AIFollowerDefinition.h"

// GPS
TinyGPS gps;

//App input
int state;

// Master Enable
bool enabled = false;

// Serial components
SoftwareSerial bluetoothSerial(BLUETOOTH_RX_PIN,BLUETOOTH_TX_PIN); //Bluetooth Module
SoftwareSerial nss(GPS_RX_PIN, GPS_TX_PIN);                       //GPS Module

/* Compass */
HMC5883L compass;

GeoLoc checkGPS() {
  Serial.println("Reading onboard GPS: ");
  bool newdata = false;
  unsigned long start = millis();
  while (millis() - start < GPS_UPDATE_INTERVAL) {
    if (feedgps())
      newdata = true;
  }
  if (newdata) {
   
    return gpsdump(gps);
  }

  GeoLoc coolerLoc;
  coolerLoc.lat = 0.0;
  coolerLoc.lon = 0.0;
  
  return coolerLoc;
}

// Get and process GPS data
GeoLoc gpsdump(TinyGPS &gps) {
  float flat, flon;
  unsigned long age;
  
  gps.f_get_position(&flat, &flon, &age);

  GeoLoc coolerLoc;
  coolerLoc.lat = flat;
  coolerLoc.lon = flon;

 // Serial.print(flat, 7); Serial.print(", "); Serial.println(flat, 7);

  return coolerLoc;
}

// Feed data as it becomes available 
bool feedgps() {
  while (nss.available()) {
   // Serial.write(nss.read());
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

float geoBearing(struct GeoLoc &a, struct GeoLoc &b) {
  float y = sin(b.lon-a.lon) * cos(b.lat);
  float x = cos(a.lat)*sin(b.lat) - sin(a.lat)*cos(b.lat)*cos(b.lon-a.lon);
  return atan2(y, x) * RADTODEG;
}

float geoDistance(struct GeoLoc &a, struct GeoLoc &b) {
  const float R = 6371000; // km
  float p1 = a.lat * DEGTORAD;
  float p2 = b.lat * DEGTORAD;
  float dp = (b.lat-a.lat) * DEGTORAD;
  float dl = (b.lon-a.lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2); // law of haversine
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}

float geoHeading() {
  Vector norm = compass.readNormalize();
// Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);
  
  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (-2.0 + (-22.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }
  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 
  
  // Map to -180 - 180
  while (headingDegrees < -180) headingDegrees += 360;
  while (headingDegrees >  180) headingDegrees -= 360;

  return headingDegrees;
}

void setSpeedMotorA(int speed) {
  digitalWrite(MOTOR_A_IN_1_PIN, HIGH);
  digitalWrite(MOTOR_A_IN_2_PIN, LOW);
  
  // set speed to 200 out of possible range 0~255
  analogWrite(MOTOR_A_EN_PIN, speed);
}

void setSpeedMotorB(int speed) {
  digitalWrite(MOTOR_B_IN_1_PIN, HIGH);
  digitalWrite(MOTOR_B_IN_2_PIN, LOW);
  
  // set speed to 200 out of possible range 0~255
  analogWrite(MOTOR_B_EN_PIN, speed);
}

void setSpeed(int speed)
{
  // this function will run the motors in both directions at a fixed speed
  // turn on motor A
  setSpeedMotorA(speed);

  // turn on motor B
  setSpeedMotorB(speed);
}

void stop() {
  // now turn off motors
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, LOW);  
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, LOW);
}

void drive(int distance, float turn) {
  int fullSpeed = 160;
  int stopSpeed = 0;

  // drive to location
  int s = fullSpeed;
  if ( distance < 8 ) {
    int wouldBeSpeed = s - stopSpeed;
    wouldBeSpeed *= distance / 8.0f;
    s = stopSpeed + wouldBeSpeed;
  }
  
  int autoThrottle = constrain(s, stopSpeed, fullSpeed);
  autoThrottle = 160;

  float t = turn;
  while (t < -180) t += 360;
  while (t >  180) t -= 360;
  
  Serial.print("turn: ");
  Serial.println(t);
  Serial.print("original: ");
  Serial.println(turn);
  
  float t_modifier = (180.0 - abs(t)) / 180.0;
  float autoSteerA = 1;
  float autoSteerB = 1;

  if (t < 0) {
    autoSteerB = t_modifier;
  } else if (t > 0){
    autoSteerA = t_modifier;
  }

  Serial.print("steerA: "); Serial.println(autoSteerA);
  Serial.print("steerB: "); Serial.println(autoSteerB);

  int speedA = (int) (((float) autoThrottle) * autoSteerA);
  int speedB = (int) (((float) autoThrottle) * autoSteerB);
  
  setSpeedMotorA(speedA);
  setSpeedMotorB(speedB);
}

void driveTo(struct GeoLoc &loc, int timeout) {
  nss.listen();
  GeoLoc coolerLoc = checkGPS();
  bluetoothSerial.listen();
 
float d = 0;
 //Serial.println(geoHeading());

  if (coolerLoc.lat != 0 && coolerLoc.lon != 0 && enabled) {
      do{
           nss.listen();
           coolerLoc = checkGPS();
           bluetoothSerial.listen();
           d = geoDistance(coolerLoc, loc);
           Serial.print("Phone - "); Serial.print(loc.lat,7); Serial.print(", "); Serial.println(loc.lon,7);
           Serial.print("Module - ");  Serial.print(coolerLoc.lat,7); Serial.print(", "); Serial.println(coolerLoc.lon,7);
            
           float t = geoBearing(coolerLoc, loc) - geoHeading();
      
          Serial.print("Distance: ");
          Serial.println(geoDistance(coolerLoc, loc));
        
          Serial.print("Bearing: ");
          Serial.println(geoBearing(coolerLoc, loc));
    
          Serial.print("heading: ");
          Serial.println(geoHeading());
    
          Serial.print("t: ");
          Serial.println(t);
    
          
          drive(d, t);
          timeout -= 1;
          
        }while(d > 3.0 && enabled && timeout>0);

          stop();
  }
}
 


void setupCompass() {
   /* Initialise the compass */

   while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
      // Set measurement range
    compass.setRange(HMC5883L_RANGE_1_3GA);
  
    // Set measurement mode
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
  
    // Set data rate
    compass.setDataRate(HMC5883L_DATARATE_30HZ);
  
    // Set number of samples averaged
    compass.setSamples(HMC5883L_SAMPLES_8);
  
    // Set calibration offset. See HMC5883L_calibration.ino
    compass.setOffset(-65, -210);
  

}

void setup()
{
  // Motor pins
  pinMode(MOTOR_A_EN_PIN, OUTPUT);
  pinMode(MOTOR_B_EN_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_2_PIN, OUTPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //Debugging via serial
  Serial.begin(4800);

// Compass
  setupCompass();
  
  //GPS
  nss.begin(9600);

  //Bluetooth
  bluetoothSerial.begin(9600);
}

void compassTest(){
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (-2.0 + (-22.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 

  // Output
  Serial.print(" Heading = ");
  Serial.print(heading);
  Serial.print(" Degress = ");
  Serial.print(headingDegrees);
  Serial.println();

  delay(100);
  }

void loop()
{ 

 //compassTest();
  bluetoothSerial.listen();

  if(bluetoothSerial.available() > 0 && enabled == false ){
    state = bluetoothSerial.read();
 //  Serial.println(state);

  }

  if (state == 70) { //forward "F"
     
      analogWrite(MOTOR_A_EN_PIN, 160);
      analogWrite(MOTOR_B_EN_PIN, 160);
      digitalWrite(MOTOR_A_IN_1_PIN, HIGH);
      digitalWrite(MOTOR_A_IN_2_PIN, LOW);  
      digitalWrite(MOTOR_B_IN_1_PIN, HIGH);
      digitalWrite(MOTOR_B_IN_2_PIN, LOW); 
      
    }

    if (state == 76) { //left "F"
      analogWrite(MOTOR_B_EN_PIN, 160);
      digitalWrite(MOTOR_B_IN_1_PIN, HIGH);
      digitalWrite(MOTOR_B_IN_2_PIN, LOW); 
   
    }


    if (state == 82) {//right "R"
      
      analogWrite(MOTOR_A_EN_PIN, 160);
       digitalWrite(MOTOR_A_IN_1_PIN, HIGH);
      digitalWrite(MOTOR_A_IN_2_PIN, LOW);  
    }
    
    if (state == 66) { //reverse/back "B"
      analogWrite(MOTOR_A_EN_PIN, 160);
      analogWrite(MOTOR_B_EN_PIN, 160);
      digitalWrite(MOTOR_A_IN_1_PIN, LOW);
      digitalWrite(MOTOR_A_IN_2_PIN, HIGH);  
      digitalWrite(MOTOR_B_IN_1_PIN, LOW);
      digitalWrite(MOTOR_B_IN_2_PIN, HIGH); 
    }
    
    if (state == 83) { //stop "S"
      stop();
      enabled = false;
    }

    if (state == 69){ // "Enabled E"
      delay(100);
      enabled = true;
      ReceiveGPSData();
    }
  
}


void ReceiveGPSData(){
  
bool isPhoneGPSready = false;
bool isGPSLonstreaming = false;
bool isGPSLatstreaming = false;
int stateRec;

  String bilat,bilon;
  
  while(isPhoneGPSready == false){
    while(bluetoothSerial.available() > 0){
      stateRec = bluetoothSerial.read();
      //  Serial.println(stateRec);
          if (stateRec == 83){
            stop();
            return;
            }

          if (stateRec == 64){
              isGPSLonstreaming = true;
              isGPSLatstreaming = false;

          }
          if (stateRec == 35){
              isGPSLatstreaming = true;
              isGPSLonstreaming = false;

          }

          if (isGPSLonstreaming == true && stateRec != 64 ){
              bilon += char(stateRec);

          }
          if (isGPSLatstreaming == true && stateRec != 35){
              
            if (stateRec == 36){
              isPhoneGPSready = true;
              break;
             
            }
            bilat += char(stateRec);
          }  
      }
    }

//  Serial.print(bilat); Serial.print(", "); Serial.println(bilon);

  GeoLoc phoneLoc;
  phoneLoc.lat = bilat.toFloat();
  phoneLoc.lon = bilon.toFloat();

 if (phoneLoc.lat != 0 && phoneLoc.lon != 0 ) {
 
   driveTo(phoneLoc, GPS_STREAM_TIMEOUT);
 }
}
