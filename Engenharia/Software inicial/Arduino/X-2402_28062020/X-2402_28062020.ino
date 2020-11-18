#include <math.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <PString.h>

#include "Main_GPS.cpp"
GPS gps;
#include "WayPoints.cpp"
WP wp;
#include "Rudder.cpp"
Rudder rudder;
#include "Decode_NMEA.cpp"
DECODE_NMEA Decode_NMEA;
#include "Control.cpp"
Control control;

#define RUDDER_PIN1 11 // leme EB
#define RUDDER_PIN2 13 // leme BB

#define RUDDER_1_TRIM 94 //VALORES PARA CENTRAR OS LEMES (TRIM)
#define RUDDER_2_TRIM 102

#define MOTOR1_PIN 10 // motor EB esta
#define MOTOR2_PIN 12 // motor BB

#define MOTOR_PARADO 1500

#define NAV_LIGHTS_PIN 9 // PIN9 - NAVIGATION LIGHTS

//
// PINS 2/3 - RELES POTENCIA FLUTUADORS
//


//*******************************************
float DistCalc = 1000; // float dist2wp = 0; // dist2wp is the distance (in meters) to the next Waypoint2
float AziCalc = 270;
float Manual_head = 320;
float XTD = 0;

float head = 0;
float roll = 0;
float pitch = 0;

bool autonomous = true;

bool sail_stop = false;
bool sail_auto = true;
bool sail_upwind = false;

bool State1 = true;
bool State2 = false;
bool State3 = false;
bool State4 = false;
bool State5 = false;

long double millisBS = 0;
long double millis_sail = 0;


String Comand;

double timer = 0;  //time it takes for the void loop run
float frequency = 0; //frequency of the code

File myFile;
Servo servo_rudder1;
Servo servo_rudder2;

Servo motor1;
Servo motor2;


void setup() {

  Serial.begin(57600); // connect serial
  Serial.setTimeout(100);
  Serial1.begin(57600); // connect Comunications
  Serial1.setTimeout(100);
  Serial2.begin(9600); // connect gps sensor
  Serial3.begin(4800); // LT1000

  Serial.println("Sistema a iniciar, seja bem-vindo");

  servo_rudder1.attach(RUDDER_PIN1);    // the rudder is connected to pin 12
  servo_rudder2.attach(RUDDER_PIN2);

  servo_rudder1.write(RUDDER_1_TRIM);
  servo_rudder2.write(RUDDER_2_TRIM);

  Serial.print("Leme 1 na posição: "); Serial.println(RUDDER_1_TRIM);
  Serial.print("Leme 2 na posição: "); Serial.println(RUDDER_2_TRIM);

  motor1.attach(MOTOR1_PIN);
  motor1.write(MOTOR_PARADO);

  motor2.attach(MOTOR2_PIN);
  motor2.write(MOTOR_PARADO);

  pinMode(NAV_LIGHTS_PIN, OUTPUT);
  digitalWrite(NAV_LIGHTS_PIN, HIGH);

  Serial.print("Motores Parados: "); Serial.println(MOTOR_PARADO);
  wp.Update();
  Serial.print("Lista de wp's: ");
  wp.wp_list();
  wp.last_wp_number = wp.total_wp_number;

  Serial.println("Tudo ok, vamos iniciar manobra.");
  motor1.write(control.motor_eb_speed);
  motor2.write(control.motor_bb_speed);
}


void loop() {

  Decode_NMEA.Update();
  ReadComand();
  gps.Update();
  DistCalc = gps.CalcDist(Decode_NMEA.lat, Decode_NMEA.lon, (wp.wplat[wp.number]), (wp.wplon[wp.number])); // Calcula a distancia entre os Wp's e a posição atual
  AziCalc = gps.CalcAzi(Decode_NMEA.lat, Decode_NMEA.lon, (wp.wplat[wp.number]), (wp.wplon[wp.number])); // Calcula o Azimute entre os Wp's e a posição atual
  XTD = gps.CalcCrossTrackError((wp.wplat[wp.last_wp_number]), (wp.wplon[wp.last_wp_number]), Decode_NMEA.lat, Decode_NMEA.lon, (wp.wplat[wp.number]), (wp.wplon[wp.number])  );
  Update_GCS(1000); //250

  if (State1) { // Semi_Autonomous - head
    servo_rudder1.write(rudder.UpdateRudder1(Decode_NMEA.head, Manual_head, RUDDER_1_TRIM));
    servo_rudder2.write(rudder.UpdateRudder2(Decode_NMEA.head, Manual_head, RUDDER_2_TRIM));
  }
  if (State2) { // Autonomous - head
    servo_rudder1.write(rudder.UpdateRudder1(Decode_NMEA.head, AziCalc, RUDDER_1_TRIM));
    servo_rudder2.write(rudder.UpdateRudder2(Decode_NMEA.head, AziCalc, RUDDER_2_TRIM));
  }
  if (State3) { // Semi_Autonomous - COG
    servo_rudder1.write(rudder.UpdateRudder1(Decode_NMEA.COG, Manual_head, RUDDER_1_TRIM));
    servo_rudder2.write(rudder.UpdateRudder2(Decode_NMEA.COG, Manual_head, RUDDER_2_TRIM));
  }
  if (State4) { // Autonomous - COG
    servo_rudder1.write(rudder.UpdateRudder1(Decode_NMEA.COG, AziCalc, RUDDER_1_TRIM));
    servo_rudder2.write(rudder.UpdateRudder2(Decode_NMEA.COG, AziCalc, RUDDER_2_TRIM));
  }
  if (State5) { // EXPERIMENTAL
    control.Update_head_to_go(Decode_NMEA.head, Decode_NMEA.COG, XTD, AziCalc, DistCalc );
    servo_rudder1.write(rudder.UpdateRudder1(Decode_NMEA.head, control.head_to_go, RUDDER_1_TRIM));
    servo_rudder2.write(rudder.UpdateRudder2(Decode_NMEA.head, control.head_to_go, RUDDER_2_TRIM));
  }

  wp.test(DistCalc);

  frequency = (millis() - timer);
  timer = millis();
}

void ReadComand() {
  String validComand = "Invalid";
  if (Serial1.available() > 0)
    Comand = Serial1.readString();
  if (Serial.available() > 0)
    Comand = Serial.readString();

  if (Comand == "") {
  }
  else {

    Comand = Comand.substring(0, Comand.length() - 1);
    Serial.print("Comando: "); Serial.println(Comand);
    Serial1.print("Comando: "); Serial1.println(Comand);

    if (Comand == "STAT1") { // Semi_Autonomous - head
      State1 = true;
      State2 = false;
      State3 = false;
      State4 = false;
      State5 = false;
      validComand = "Valid";
      autonomous = false;
    }
    if (Comand == "STAT2") {  // Autonomous - head
      State1 = false;
      State2 = true;
      State3 = false;
      State4 = false;
      State5 = false;
      autonomous = true;
      validComand = "Valid";
    }
    if (Comand == "STAT3") {  // Semi_Autonomous - COG
      State1 = false;
      State2 = false;
      State3 = true;
      State4 = false;
      State5 = false;
      autonomous = false;
      validComand = "Valid";
    }
    if (Comand == "STAT4") {  // Autonomous - COG
      State1 = false;
      State2 = false;
      State3 = false;
      State4 = true;
      State5 = false;
      autonomous = true;
      validComand = "Valid";
    }
    if (Comand == "STAT5") {  // Autonomous - COG
      State1 = false;
      State2 = false;
      State3 = false;
      State4 = false;
      State5 = true;
      autonomous = true;
      validComand = "Valid";
    }

    if (Comand == "reset") {
      control.software_Reset();
    }

    if (Comand == "w+") {
      wp.last_wp_number = wp.number;
      wp.number++;
      if (wp.wplat[wp.number] == 0) {
        wp.number = 1;
      }
      Serial.println("wp mais");
      Serial1.println("wp mais");
      validComand = "Valid";
    }

    if (Comand == "w-") {
      wp.last_wp_number = wp.number;
      wp.number--;
      if (wp.number < 1) {
        wp.number = wp.total_wp_number;
      }
      Serial.println("wp menos");
      Serial1.println("wp menos");
      validComand = "Valid";
    }

    if (Comand == "TestGPS") {
      Serial.println("Testing GPS: Vamos parar motores e colocar lemes a meio");
      validComand = "Valid";
      servo_rudder1.write(RUDDER_1_TRIM);
      servo_rudder2.write(RUDDER_2_TRIM);

      motor1.write(1500);
      motor2.write(1500);

      gps.Test();
    }

    if (Comand.substring(0, 1) == "h" && Comand.substring(Comand.length() - 1, Comand.length()) == "*")
    {
      if (Comand.substring(1, 4).toInt() >= 0  && Comand.substring(1, 4).toInt() < 360)
      {
        Manual_head = Comand.substring(1, 4).toInt();
        Serial.print("nova proa: "); Serial.println(Comand.substring(1, 4).toInt());
        Serial1.print("nova proa: "); Serial1.println(Comand.substring(1, 4).toInt());
        validComand = "Valid";
      }
      else
      {
        Serial.println("Proa nao valida camarada!");
        Serial1.println("Proa nao valida camarada!");
      }
    }
    if (Comand.substring(0, 4) == "m_bb")
    {
      if (Comand.substring(4, 8).toInt() > 999 && Comand.substring(4, 8).toInt() < 2001)
        control.motor_bb_speed = Comand.substring(4, 8).toInt();
      motor2.write(control.motor_bb_speed);
      Serial.print("motor de bb a: "); Serial.println(Comand.substring(4, 8).toInt());
      Serial1.print("motor de bb a: "); Serial1.println(Comand.substring(4, 8).toInt());
      validComand = "Valid";
    }
    if (Comand.substring(0, 4) == "m_eb")
    {
      if (Comand.substring(4, 8).toInt() > 999 && Comand.substring(4, 8).toInt() < 2001)
        control.motor_eb_speed = Comand.substring(4, 8).toInt();
      motor1.write(control.motor_eb_speed);
      Serial.print("motor de eb a: "); Serial.println(Comand.substring(4, 8).toInt());
      Serial1.print("motor de eb a: "); Serial1.println(Comand.substring(4, 8).toInt());
      validComand = "Valid";
    }

    if (Comand == "wp_list") {
      validComand = "Valid";
      wp.wp_list();
    }
    if (Comand == "NAV_ON") {
      validComand = "Valid";
      digitalWrite(NAV_LIGHTS_PIN, HIGH);
    }
    if (Comand == "NAV_OFF") {
      validComand = "Valid";
      digitalWrite(NAV_LIGHTS_PIN, LOW);
    }


    Comand = "";
    Serial.println(validComand);
    Serial1.println(validComand);
  }
}

void Update_GCS(int update_time) {
  if (millis() - millisBS > update_time) {
    millisBS = millis();
    Serial.println("Frequencia: " + String(frequency));
    //________________________________________________________________________________//
    //COMPASS LT-1000
    Serial.println("LAT NMEA: " + String(Decode_NMEA.lat));
    Serial.println("LON NMEA: " + String(Decode_NMEA.lon));
    Serial.println("HEAD: " + String(Decode_NMEA.head));
    Serial.println("COG: " + String(Decode_NMEA.COG));
    //________________________________________________________________________________//
    //COMPASS NMEA
    char NMEA_compass [35];
    byte csm; //checksum compass
    PString strm(NMEA_compass, sizeof(NMEA_compass));
    strm.print("$HDM,");
    strm.print(lround(Decode_NMEA.head));  // lround simply rounds out the decimal, since a single degree is fine enough of a resolution
    strm.print(",M*");
    csm = checksum(NMEA_compass);
    if (csm < 0x10) strm.print('0');
    strm.print(csm, HEX);
    Serial1.println(NMEA_compass);
    Serial.println(NMEA_compass);
    //________________________________________________________________________________//
    //COMPASS CEOV USV STAT
    char CEOV_USV_STAT [75];
    byte csm_stat; //checksum compass
    PString strm_stat(CEOV_USV_STAT, sizeof(CEOV_USV_STAT));
    strm_stat.print("$stat,");
    strm_stat.print(autonomous);  // lround simply rounds out the decimal, since a single degree is fine enough of a resolution
    strm_stat.print(",");
    strm_stat.print(control.motor_eb_speed);  // lround simply rounds out the decimal, since a single degree is fine enough of a resolution
    strm_stat.print(",");
    strm_stat.print(control.motor_bb_speed);  // lround simply rounds out the decimal, since a single degree is fine enough of a resolution
    strm_stat.print(",");
    if (Manual_head > 99) {
      strm_stat.print(Manual_head);  // lround simply rounds out the decimal, since a single degree is fine enough of a resolution
    }
    if (Manual_head > 9 && Manual_head < 100) {
      strm_stat.print("0");
      strm_stat.print(Manual_head);  // lround simply rounds out the decimal, since a single degree is fine enough of a resolution
    }
    if (Manual_head >= 0 && Manual_head < 10) {
      strm_stat.print("00");
      strm_stat.print(Manual_head);  // lround simply rounds out the decimal, since a single degree is fine enough of a resolution
    }
    strm_stat.print(",M*");
    csm_stat = checksum(CEOV_USV_STAT);
    if (csm_stat < 0x10) strm_stat.print('0');
    strm_stat.print(csm_stat, HEX);
    Serial1.println(CEOV_USV_STAT);
    Serial.println(CEOV_USV_STAT);
    //________________________________________________________________________________//
    //MOTOR NMEA
    char NMEA_MOTOR_EB [23];
    byte cs_motor; //checksum compass
    PString strm_motor(NMEA_MOTOR_EB, sizeof(NMEA_MOTOR_EB));
    strm_motor.print("$rpm,E,1,");
    strm_motor.print(control.motor_eb_speed);  // lround simply rounds out the decimal, since a single degree is fine enough of a resolution
    strm_motor.print(".0,,A*");
    cs_motor = checksum(NMEA_MOTOR_EB);
    if (csm < 0x10) strm.print('0');
    strm_motor.print(cs_motor, HEX);
    Serial1.println(NMEA_MOTOR_EB);
    Serial.println(NMEA_MOTOR_EB);
    //________________________________________________________________________________//
    //GPS NMEA
    char NMEA_gps_GPRMC [100];
    byte cs_GPRMC; //checksum compass
    PString strm1(NMEA_gps_GPRMC, sizeof(NMEA_gps_GPRMC));
    strm1.print("$GPRMC,");
    strm1.print(String(Decode_NMEA.Time));
    strm1.print(",A,");
    strm1.print(Decode_NMEA.NMEA_Lat + ",");
    strm1.print(Decode_NMEA.lat_Signal + ",");
    strm1.print(Decode_NMEA.NMEA_Lon + ",");
    strm1.print(Decode_NMEA.lon_Signal + ",");
    strm1.print(Decode_NMEA.Speed_Kts);
    strm1.print(",");
    strm1.print(Decode_NMEA.head);
    strm1.print(",");
    strm1.print(gps.date);
    strm1.print(",002.1,W*");
    cs_GPRMC = checksum(NMEA_gps_GPRMC);
    if (cs_GPRMC < 0x10) strm1.print('0');
    strm1.print(cs_GPRMC, HEX);
    Serial1.println(NMEA_gps_GPRMC);
    Serial.println(NMEA_gps_GPRMC);
    //______________________________________________________________________________//
    //*WIND NMEA
    /*     char NMEA_wind [23]; //ex: $NRMWV,179,R,55,N,A*1e
         byte cs_wind; //checksum compass
         PString strm2(NMEA_wind, sizeof(NMEA_wind));
         strm2.print("$NRMWV,");
         strm2.print(wind.Direction);
         strm2.print(",R,");
         strm2.print(wind.Speed);
         strm2.print(",N,A*");
         cs_wind = checksum(NMEA_wind);
         if (cs_wind < 0x10) strm2.print('0');
         strm2.print(cs_wind, HEX);
         Serial2.println(NMEA_wind);
         Serial.println(NMEA_wind);*/
    //______________________________________________________________________________//
    //RUDDER NMEA
    char NMEA_rudder [50];
    byte cs_rudder; //checksum compass
    PString strm3(NMEA_rudder, sizeof(NMEA_rudder));
    strm3.print("$RSA,");
    strm3.print(rudder.NMEA_rudder);
    strm3.print(",A,,*");
    cs_rudder = checksum(NMEA_rudder);
    if (cs_rudder < 0x10) strm3.print('0');
    strm3.print(cs_rudder, HEX);
    Serial1.println(NMEA_rudder);
    Serial.println(NMEA_rudder);
    //______________________________________________________________________________//
    //CEOV WP_GO
    char WP_INFO [80];
    byte cs_WP_INFO; //checksum compass
    PString strm_WP_INFO(WP_INFO, sizeof(WP_INFO));
    strm_WP_INFO.print("$WP_INFO,");
    strm_WP_INFO.print(wp.number);
    strm_WP_INFO.print(",");
    strm_WP_INFO.print(DistCalc);
    strm_WP_INFO.print(",");
    strm_WP_INFO.print(AziCalc);
    strm_WP_INFO.print(",");
    strm_WP_INFO.print(gps.GPS_head);
    strm_WP_INFO.print(",");
    strm_WP_INFO.print(gps.Speed_kts);
    strm_WP_INFO.print(",*");
    cs_WP_INFO = checksum(WP_INFO);
    if (cs_WP_INFO < 0x10) strm_WP_INFO.print('0');
    strm_WP_INFO.print(cs_WP_INFO, HEX);
    Serial1.println(WP_INFO);
    Serial.println(WP_INFO);
    //______________________________________________________________________________//
    //PITCH ROLL NMEA
    char shrSentence [50]; //ex:$IIXDR,A,5.0,,PTCH,A,12.0,,ROLL,*hh<CR><LF>
    byte csp;
    PString strp(shrSentence, sizeof(shrSentence));
    strp.print("$IIXDR,A,");
    strp.print(pitch, 1);
    strp.print(",,PTCH,A,");
    strp.print(roll, 1);
    strp.print(",,ROLL,*");
    csp = checksum(shrSentence);
    if (csp < 0x10) strp.print('0');
    strp.print(csp, HEX);
    Serial1.println(shrSentence);
    Serial.println(shrSentence);

  }
}

byte checksum(char* str) { //Checksum function
  byte cs = 0;
  for (unsigned int n = 1; n < strlen(str) - 1; n++)
  {
    cs ^= str[n];
  }
  return cs;
}
