#include <Servo.h>

#define RUDDER_PIN1 11 // leme EB
#define RUDDER_PIN2 13 // leme BB

#define RUDDER_1_TRIM 100 //VALORES PARA CENTRAR OS LEMES (TRIM)
#define RUDDER_2_TRIM 100

#define MOTOR1_PIN 10 // motor EB esta
#define MOTOR2_PIN 12 // motor BB

#define MOTOR_PARADO 1500

#define NAV_LIGHTS_PIN 9 // PIN9 - NAVIGATION LIGHTS

#define MOTOR_BB_RED 8
#define MOTOR_BB_GREEN 7
#define MOTOR_BB_BLUE 6

#define MOTOR_EB_RED 5
#define MOTOR_EB_GREEN 4
#define MOTOR_EB_BLUE 3


//
// PINS 2/3 - RELES POTENCIA FLUTUADORS
//


double timer = 0;  //time it takes for the void loop run
float frequency = 0; //frequency of the code

Servo servo_rudder1;
Servo servo_rudder2;

Servo motor1;
Servo motor2;


int16_t motor_bb = 1500;
int16_t motor_eb = 1500;
int16_t rudder_bb = 1500;
int16_t rudder_eb = 1500;

String Comand;

/*
    if (channel == 'm')  {
    motor_bb = value;
    } else if (channel == 'M')  {
    motor_eb = value;
    } else if (channel == 'l')  {
    rudder_bb = value;
    } else if (channel == 'L')  {
    rudder_eb = value;
*/
void ReadComand() {
    String validComand = "Invalid";
    char buffer[10];
    if (Serial1.available() > 0)
        Comand = Serial1.readString();
    if (Serial.available() > 0){
        //Comand = Serial.readString();
        Serial.readBytesUntil( '*', buffer, 9);
        //Serial.println(buffer);
        Comand = buffer;
        //Serial.print(Comand); 
        Comand = Comand.substring(0, 5);
        //Serial.readBytes(buffer, 9);
        //Serial.flush();
    }

    if (Comand == "") {
    }
    else {

        //Comand = Comand.substring(0, Comand.length() - 1);
        Serial.print("Comando: "); Serial.print(Comand);Serial.print("da "); 
        Serial.print("Comando substring: "); 
        Serial.print(0);
        Serial.print(buffer[0]);
        Serial.print(" ");
        Serial.print(1);
        Serial.print(buffer[1]);
        Serial.print(" ");
        Serial.print(2);
        Serial.print(buffer[2]);
        Serial.print(" ");
        Serial.print(3);
        Serial.print(buffer[3]);
        Serial.print("te"); 

        //if (Comand.substring(Comand.length() - 1, Comand.length()) == "*")
        {
            if (buffer[0] == 'm')
            {
                
                int value = Comand.substring(1, 5).toInt();
                Serial.println(value);
                if (value >= 1000  && value <= 2000)
                {
                    motor_bb = value;
                    Serial.print("novo motor bombordo: ");
                    Serial.println(value);
                    validComand = "Valid";
                }
                else
                {
                    Serial.println("Motor nao valido camarada!");
                    Serial1.println("Motor nao valido camarada!");
                }
            }
            if (buffer[0] == 'M')
            {
                int value = Comand.substring(1, 5).toInt();
                if (value >= 1000  && value <= 2000)
                {
                    motor_eb = value;
                    Serial.print("novo motor estibordo: ");
                    Serial.println(value);
                    validComand = "Valid";
                }
                else
                {
                    Serial.println("Motor nao valido camarada!");
                    Serial1.println("Motor nao valido camarada!");
                }
            }
            if (buffer[0] == 'l')
            {
                int value = Comand.substring(1, 5).toInt();
                if (value >= 1000  && value <= 2000)
                {
                    rudder_bb = value;
                    Serial.print("leme bombordo: ");
                    Serial.println(value);
                    validComand = "Valid";
                }
                else
                {
                    Serial.println("Leme nao valido camarada!");
                    Serial1.println("Leme nao valido camarada!");
                }
            }
            if (buffer[0] == 'L')
            {
                int value = Comand.substring(1, 5).toInt();
                if (value >= 1000  && value < 2000)
                {
                    rudder_eb = value;
                    Serial.print("leme estibordo: ");
                    Serial.println(value);
                    validComand = "Valid";
                }
                else
                {
                    Serial.println("Leme nao valido camarada! ");
                    Serial.print(value);
                    Serial1.println("Leme nao valido camarada!");
                }
            }
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

//DO NOT KEEP THIS
void motor_to_leds(int mbb, int meb) {
    int r = 0, g = 0, b = 0;

    r = 100 - abs((mbb - 1500) / 5);
    g = (mbb > 1500) * (mbb - 1500) / 5;
    b = (mbb < 1500) * (1500 - mbb) / 5;
    analogWrite(MOTOR_BB_RED, r);
    analogWrite(MOTOR_BB_GREEN, g);
    analogWrite(MOTOR_BB_BLUE, b);

    r = 100 - abs((meb - 1500) / 5);
    g = (meb > 1500) * (meb - 1500) / 5;
    b = (meb < 1500) * (1500 - meb) / 5;
    analogWrite(MOTOR_EB_RED, r);
    analogWrite(MOTOR_EB_GREEN, g);
    analogWrite(MOTOR_EB_BLUE, b);

}

// the setup function runs once when you press reset or power the board
void setup() {
    /*
        Serial1.begin(57600); // connect Comunications
        Serial1.setTimeout(100);
        Serial2.begin(9600); // connect gps sensor
        Serial3.begin(4800); // LT1000

        motor1.attach(MOTOR1_PIN);
        motor1.write(MOTOR_PARADO);

        motor2.attach(MOTOR2_PIN);
        motor2.write(MOTOR_PARADO);

        pinMode(NAV_LIGHTS_PIN, OUTPUT);
        digitalWrite(NAV_LIGHTS_PIN, HIGH);
    */
    Serial.begin(115200);
    Serial.println("Sistema a iniciar, seja bem-vindo");

    servo_rudder1.attach(RUDDER_PIN1);    // the rudder is connected to pin 12 or not..
    servo_rudder2.attach(RUDDER_PIN2);

    servo_rudder1.write(RUDDER_1_TRIM);
    servo_rudder2.write(RUDDER_2_TRIM);

    Serial.print("Leme 1 na posição: "); Serial.println(RUDDER_1_TRIM);
    Serial.print("Leme 2 na posição: "); Serial.println(RUDDER_2_TRIM);

    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(MOTOR_BB_RED, OUTPUT);
    pinMode(MOTOR_BB_GREEN, OUTPUT);
    pinMode(MOTOR_BB_BLUE, OUTPUT);
    pinMode(MOTOR_EB_RED, OUTPUT);
    pinMode(MOTOR_EB_GREEN, OUTPUT);
    pinMode(MOTOR_EB_BLUE, OUTPUT);

//    Serial.bufferUntil("\n");
}

// the loop function runs over and over again forever
void loop() {
    ReadComand();
    /*
        motor1.write(control.motor_eb_speed);
        motor2.write(control.motor_bb_speed);

    */

    servo_rudder1.write(rudder_bb);
    servo_rudder2.write(rudder_eb);

    motor_to_leds(motor_bb, motor_eb);


    /*
        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(motor_eb);                       // wait for a second
        digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
        delay(motor_bb);                       // wait for a second
    */


    frequency = (millis() - timer);
    timer = millis();
}
