#include "Arduino.h"

class Control {
  public:
    //********************************************
    float head_to_go = 270;
    int correction_factor = 45;

    int help_rotate_factor = 10;

    int motor_eb_speed = 1500;
    int motor_bb_speed = 1500;

    int correct_motor_eb_speed = 1500;
    int correct_motor_bb_speed = 1500;

    //********************************************

    void Update_head_to_go(float head, float COG, float XTD, float AziCalc, float DistCalc) {
      head_to_go = AziCalc + angle_dif(head, COG) + (correction_factor * ( XTD / DistCalc  ));
    }

    float help_rotate(float head) {

      float dif = angle_dif(head, head_to_go );

      if (abs(dif) > 30) {
        correct_motor_eb_speed = motor_eb_speed + (dif * help_rotate_factor);
        correct_motor_bb_speed = motor_bb_speed - (dif * help_rotate_factor);
      }

      else {
        correct_motor_eb_speed = motor_eb_speed;
        correct_motor_bb_speed = motor_bb_speed;
      }

      if (correct_motor_eb_speed > 2000) {
        correct_motor_eb_speed = 2000;
      }
      if (correct_motor_eb_speed < 1000) {
        correct_motor_eb_speed = 1000;
      }
      if (correct_motor_bb_speed > 2000) {
        correct_motor_bb_speed = 2000;
      }
      if (correct_motor_bb_speed < 1000) {
        correct_motor_bb_speed = 1000;
      }
    }


    float angle_dif(int a, int b) {
      int d = abs(a - b) % 360;
      int r = d > 180 ? 360 - d : d;

      //calculate sign
      int sign = (a - b >= 0 && a - b <= 180) || (a - b <= -180 && a - b >= -360) ? 1 : -1;
      r *= sign;
      return (r);
    }


    void software_Reset() { // Restarts program from beginning but does not reset the peripherals and registers
      Serial.println("going for reset");
      asm volatile ("  jmp 0");
    }
};
