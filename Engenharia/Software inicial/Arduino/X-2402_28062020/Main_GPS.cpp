#include "Arduino.h"
#include <TinyGPS.h>

class GPS {
    TinyGPS gps_nr1;
  public:
    //********************************************
    long lat, lon;
    float GPS_head;
    float Speed_kts;
    unsigned long fix_age;
    unsigned long time, date;
    unsigned long time_GPS;
    bool GPS_LOCK = false;
    float NMEA_lat;
    float NMEA_lon;
    float LAT_NMEA;
    float LON_NMEA;
    //********************************************
    void Update() {

      while (Serial2.available()) {
        if (gps_nr1.encode(Serial2.read())) {
          gps_nr1.get_datetime(&date, &time, &fix_age);
          gps_nr1.get_position(&lat, &lon, &fix_age);
          Speed_kts = gps_nr1.f_speed_knots();
          GPS_head = gps_nr1.f_course();
        }

        if (fix_age == TinyGPS::GPS_INVALID_AGE) {
          GPS_LOCK = false;
        }
        else if (lat < 1) {
          GPS_LOCK = false;
        }
        else {
          GPS_LOCK = true;
        }
      }


      NMEA_lat = (lat / 100000.0000000);
      //   Serial.print("NMEA: ")  ; Serial.println(NMEA_lat,6);
      NMEA_lon = lon / 100000.000000;

      if (NMEA_lon < 0)
        NMEA_lon *= -1;

      //   Serial.print("NMEA lon : ")  ; Serial.println(NMEA_lon,6);


      LAT_NMEA = Convert_GGmm_dddddd(NMEA_lat);
      LON_NMEA = Convert_GGmm_dddddd(NMEA_lon);
    }


    float Convert_GGmm_dddddd(float coordenada) {
      int x =  coordenada;
      float y = (coordenada - x) * 60;
      String z = String(x);
      if (y < 10) {
        z = z + "0" + String(y, 6);
      }
      else {
        z = z + String(y, 6);
      }
      float w = z.toFloat();
      //w = w*100;
      return w;
    }



    float CalcDist(float flat1, float flon1, float x2lat, float x2lon) {
      // flat1 = our current latitude. flat is from the gps data.
      // flon1 = our current longitude. flon is from the fps data.

      flat1 = flat1 / 100;
      flon1 = flon1 / 100;

      float dist_calc = 0;
      float dist_calc2 = 0;
      float diflat = 0;
      float diflon = 0;

      diflat = radians(x2lat - flat1);
      flat1 = radians(flat1);
      x2lat = radians(x2lat);
      diflon = radians((x2lon) - (flon1));
      dist_calc = (sin(diflat / 2.0) * sin(diflat / 2.0));
      dist_calc2 = cos(flat1);
      dist_calc2 *= cos(x2lat);
      dist_calc2 *= sin(diflon / 2.0);
      dist_calc2 *= sin(diflon / 2.0);
      dist_calc += dist_calc2;
      dist_calc = (2 * atan2(sqrt(dist_calc), sqrt(1.0 - dist_calc)));
      dist_calc *= 6371000.0;
      return (dist_calc);
    }

    float CalcAzi(float flat1, float flon1, float x2lat, float x2lon) {

      flat1 = flat1 / 100;
      flon1 = flon1 / 100;
      flat1 = radians(flat1);

      flon1 = radians(flon1);
      x2lat = radians(x2lat);
      x2lon = radians(x2lon);

      float heading;
      heading = atan2(sin(x2lon - flon1) * cos(x2lat), cos(flat1) * sin(x2lat) - sin(flat1) * cos(x2lat) * cos(x2lon - flon1)), 2 * 3.1415926535;
      heading = heading * 180 / 3.1415926535;
      float head = heading;

      if (head < 0) {
        head += 360;
      }
      if (head > 360) {
        head -= 360;
      }

      return (head);

    }

    float CalcCrossTrackError (float lat_wp_ini, float lon_wp_ini, float flat1, float flon1, float lat_wp_fin, float lon_wp_fin) { // Calcula XTD
      float t13 = CalcAzi(lat_wp_ini*100.0, lon_wp_ini*100.0, flat1/100.0, flon1/100.0);
      float t12 = CalcAzi(lat_wp_ini*100, lon_wp_ini*100, lat_wp_fin, lon_wp_fin);
      float d13 = CalcDist(lat_wp_ini*100.0, lon_wp_ini*100.0, flat1/100.0, flon1/100.0) / 6371000.0;
      t12 = radians(t12);
      t13 = radians(t13);
      float CrossTrackError = asin(sin(d13) * sin(t13 - t12)) * 6371000.0;
      return (CrossTrackError);
    }


    void Test() {
      Serial.println("A testar GPS");
      bool a = true;
      String stringOne = "";

      while (a) {
        while (Serial2.available()) {
          char c = Serial2.read();
          if (c == '\n') {
            Serial.println(stringOne);
            stringOne = "";
          }
          else {
            stringOne =  String(stringOne + c );
          }
        }
        if (Serial.available()) {
          String Comand = Serial.readString();
          Comand = Comand.substring(0, Comand.length() - 1);
          if (Comand == "Normal") { // End Cicle, if the user send the comand "Normal";
            Serial.println("Normal mode");
            a = false;
          }
        }
      }
    }
};
