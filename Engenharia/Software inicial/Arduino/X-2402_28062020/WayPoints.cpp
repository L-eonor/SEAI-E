#include "Arduino.h"
#include <PString.h>

#define MAX_WP_NUMBER 20

class WP {
  public:
    //********************************************
    float wplat[MAX_WP_NUMBER + 1];
    float wplon[MAX_WP_NUMBER + 1];
    int total_wp_number;
    int number = 1;
    int min_dist = 3;
    int last_wp_number;
    //********************************************

    void Update() { // 5 casa decimais - precisão ~~1 metro || Primeira posição dos WP é 1 ( wplat[1], wplon[1] )

      wplat[1] = 38.64150;
      wplon[1] = -9.07884;

      wplat[2] = 38.64160;
      wplon[2] = -9.07885;

      wplat[3] = 38.64161;
      wplon[3] = -9.07869;

      wplat[4] = 38.64151;
      wplon[4] = -9.07869;

      wplat[5] = 38.64151;
      wplon[5] = -9.07884;
      
    }

    void wp_list() {
      char wp_list [600];
      byte csm_wp; //checksum compass
      PString strm_wp(wp_list, sizeof(wp_list));
      strm_wp.print("$wp,");

      int i = 1;
      while (wplat[i] != 0) {
        total_wp_number = i;
        strm_wp.print(i);
        strm_wp.print(",");
        strm_wp.print(wplat[i], 6);
        strm_wp.print(",");
        strm_wp.print(wplon[i], 6);
        strm_wp.print(",");
        i++;
      }

      strm_wp.print("M*");
      csm_wp = checksum(wp_list);
      if (csm_wp < 0x10) strm_wp.print('0');

      strm_wp.print(csm_wp, HEX);
      Serial1.println(wp_list);
      Serial.println(wp_list);
    }

    void test(float DistCalc) {

      if (DistCalc < min_dist)
        number++;

      if ((wplat[number] == 0) || (wplon[number] == 0))
        number = 1;
    }

    byte checksum(char* str) { //Checksum function
      byte cs = 0;
      for (unsigned int n = 1; n < strlen(str) - 1; n++)
      {
        cs ^= str[n];
      }
      return cs;
    }

};
