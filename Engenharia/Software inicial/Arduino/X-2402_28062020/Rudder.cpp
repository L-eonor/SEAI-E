#include "Arduino.h"
#include <math.h>

class Rudder {
  public:
    int angle = 68;
    int correction_speed = 1;
    int NMEA_rudder = 0;
    int last_rudder_order;
    int control_rudder = 98;

    int UpdateRudder1(float compass_head, int azi, int middle_rudder) {
      int delta = compass_head - azi; // calcula a diferença
      if (delta > 180) {
        delta = 360 - compass_head + azi;
        delta = -delta;
      }

      if (delta < (-180)) {
        delta = 360 - azi + compass_head;
      }

      angle = (middle_rudder - (correction_speed * delta));
      if (angle > middle_rudder + 25) {
        angle = middle_rudder + 25;
      }
      if (angle < middle_rudder - 25) {
        angle = middle_rudder - 25;
      }
      return (angle);
    }


    int UpdateRudder2(float compass_head, int azi, int middle_rudder) {
      NMEA_rudder = 0;
      int delta = compass_head - azi; // calcula a diferença
      if (delta > 180) {
        delta = 360 - compass_head + azi;
        delta = -delta;
      }

      if (delta < (-180)) {
        delta = 360 - azi + compass_head;

      }
      angle = (middle_rudder - (correction_speed * delta));
      if (angle > middle_rudder + 25) {
        angle = middle_rudder + 25;
      }
      if (angle < middle_rudder - 25) {
        angle = middle_rudder - 25;
      }

      NMEA_rudder = (-middle_rudder + angle);
      return (angle);
    }
};
