#include "Arduino.h"

class DECODE_NMEA {
  public:
    //********************************************
    int length = 100;
    char NMEA_Sentence [100];
    String NMEA_Array [30];
    char termChar = '$';
    float lat = 0;
    float lon = 0;
    float COG = 270;
    float head = 0;
    float ATM_Pressure = 0;
    float Air_Temp = 0;
    float Speed_Kts = 0;
    float Time = 0;
    float Date = 0;
    String lat_Signal = "N";
    String lon_Signal = "w";
    String NMEA_Lat = "0";
    String NMEA_Lon = "0";
    //********************************************

    void Update() {
      if (Serial3.available())
      {
        int numChars = Serial3.readBytesUntil(termChar, NMEA_Sentence, length);
        NMEA_Sentence[numChars] = '\0';
        String NMEA_String = String(NMEA_Sentence);

        if (NMEA_String.charAt((NMEA_String.length() - 5)) == '*')
        {
          char copy[NMEA_String.length() - 4];
          (NMEA_String.substring(0,  NMEA_String.length() - 4)).toCharArray(copy, NMEA_String.length() - 4);
          byte cs = checksum(copy);
          if ( String(cs, HEX).equalsIgnoreCase(NMEA_String.substring( NMEA_String.length() - 4 , NMEA_String.length() - 2))) {
            int b = 0;
            int array_p = 0;

            for (int i = 0; i < NMEA_String.length() ; i++) { //Cria array com as info NMEA
              if (1) {
                if (NMEA_String.charAt(i) ==  ',' || NMEA_String.charAt(i) == '*') {
                  NMEA_Array[array_p] = NMEA_String.substring( b , i);
                  b = i + 1;
                  array_p++;
                  if (array_p > 9)
                    break;
                }
              }
            }
            if (NMEA_String.substring(0, 5) == "HCHDT") {
              head = (NMEA_Array[1].toFloat());

            }
            else {
              if (NMEA_String.substring(0, 5) == "GNRMC") {
                lat = NMEA_Array[3].toFloat();
                lat = lat / 100;
                int i = 0;
                while (i < lat) {
                  i++;
                }
                lat = (((  ((1 - (i - lat)) / 60) * 100) + (i - 1)) * 100);
                lon = NMEA_Array[5].toFloat();
                i = 0;
                lon = lon / 100;
                while (i < lon) {
                  i++;
                }
                lon = (((  ((1 - (i - lon)) / 60) * 100) + (i - 1)) * 100);


                if (NMEA_Array[4] == "S") {
                  lat *= -1;
                  // NMEA_Array[3] = "-" + NMEA_Array[3];
                }
                if (NMEA_Array[6] == "W") {
                  lon *= -1;
                  // NMEA_Array[5] = "-" + NMEA_Array[5];
                }
                COG =  NMEA_Array[8].toFloat();
                Speed_Kts = NMEA_Array[7].toFloat();
                Time = NMEA_Array[1].toFloat();
                Date = NMEA_Array[9].toFloat();
                NMEA_Lat = NMEA_Array[3];
                lat_Signal = NMEA_Array[4];
                NMEA_Lon = NMEA_Array[5];
                lon_Signal = NMEA_Array[6];
              }
              else {
                if (NMEA_String.substring(0, 5) == "WIMDA") {
                  ATM_Pressure = (NMEA_Array[1].toFloat());
                  Air_Temp = (NMEA_Array[5].toFloat());
                }
                else {
                  Serial.print("$" + NMEA_String);
                }
              }
            }
          }
        }
      }
    }
    int string_to_float(String valor_string, int xpos, int lpos) {
      String a = valor_string.substring(xpos, lpos);
      char buf[a.length()];
      a.toCharArray(buf, a.length());
      int WVal = atof(buf);
      return WVal;
    }

    byte checksum(char* str) //Checksum function
    {
      byte cs = 0;
      for (unsigned int n = 0; n < strlen(str) ; n++)
      {
        cs ^= str[n];
      }
      return cs;
    }
};
