#include <Servo.h>
#include "channels.h"
channels_t serial_channels;

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


double timer = 0;  //time it takes for the void loop run
float frequency = 0; //frequency of the code

Servo servo_rudder1;
Servo servo_rudder2;

Servo motor1;
Servo motor2;


int16_t motor_bb{};
int16_t motor_eb{};
int16_t rudder_bb{};
int16_t rudder_eb{};

void serial_write(uint8_t b)
{
    Serial.write(b);
}

void process_serial_packet(char channel, uint32_t value, channels_t& obj)
{
    byte c;
    /*
        if (channel == 'r') {           // RFID tag
            for (c = 0; c < 4; c++) {
                //nuidPICC[c] = (value >> (c * 8)) & 0xFF;
            }
        } else if (channel == 'i')  {   // IR Sensors + Touch
            for (c = 0; c < 5; c++) {
                //IRLine.IR_values[c] = 16 * ((value >> (c * 6)) & 0x3F);
            }
            //TouchSwitch = ((value >> 31) & 1);
        } else if (channel == 'g')  {  // Control
            // Calc control
            //go = 1;
        } else if (channel == 's')  {  // Set new state
            //robot.state = value;
        else */
    if (channel == 'm')  {
        motor_bb = value;
    } else if (channel == 'M')  {
        motor_eb = value;
    } else if (channel == 'l')  {
        rudder_bb = value;
    } else if (channel == 'L')  {
        rudder_eb = value;

    } else if (channel == 't')  {
        serial_channels.sendFloat('T', frequency);              
    } else if (channel == 'p')  { // Ping
        obj.send(channel, value + 1);
    }
}

// the setup function runs once when you press reset or power the board
void setup() {
    /*
        Serial1.begin(57600); // connect Comunications
        Serial1.setTimeout(100);
        Serial2.begin(9600); // connect gps sensor
        Serial3.begin(4800); // LT1000


        servo_rudder1.attach(RUDDER_PIN1);    // the rudder is connected to pin 12
        servo_rudder2.attach(RUDDER_PIN2);

        servo_rudder1.write(RUDDER_1_TRIM);
        servo_rudder2.write(RUDDER_2_TRIM);

        motor1.attach(MOTOR1_PIN);
        motor1.write(MOTOR_PARADO);

        motor2.attach(MOTOR2_PIN);
        motor2.write(MOTOR_PARADO);

        pinMode(NAV_LIGHTS_PIN, OUTPUT);
        digitalWrite(NAV_LIGHTS_PIN, HIGH);
    */

    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    serial_channels.init(process_serial_packet, serial_write);
}

// the loop function runs over and over again forever
void loop() {

    byte serialByte;
    // Serial Port Events
    if (Serial.available() > 0) {
        serialByte = Serial.read();
        serial_channels.StateMachine(serialByte);
    }

    /*
        motor1.write(control.motor_eb_speed);
        motor2.write(control.motor_bb_speed);
        servo_rudder1.write();
        servo_rudder2.write();

    */

    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(motor_eb);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(motor_bb);                       // wait for a second
    frequency = (millis() - timer);
    timer = millis();
}
