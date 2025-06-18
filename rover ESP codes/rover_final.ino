//RUNS ON ROVER ESP

#include <Wire.h>
#include <ArduinoJson.h>  
#include <TinyGPS++.h>   

#define F_M_R 14
#define PWM_F_M_R 27
#define F_M_L 2
#define PWM_F_M_L 15
#define R_M_R 32
#define PWM_R_M_R 33
#define R_M_L 5
#define PWM_R_M_L 18

#define RXD2 16  // RX pin for GPS (connected to GPS module TX)
#define TXD2 17  // TX pin for GPS (connected to GPS module RX)

#define GPS_BAUD 9600

HardwareSerial gpsSerial(2);  
TinyGPSPlus gps;  

float linear;
float angular;
volatile float longitude;
volatile float latitude;
volatile float altitude;

unsigned long last_msg_time = 0;
const unsigned long msg_timeout = 100; 


void handleIdentification() {
    StaticJsonDocument<200> response;
    response["device_type"] = "rover";
    String output;
    serializeJson(response, output);
    Serial.println(output);
}


// ON LOW Front motors Rotate such that it moves forward
void movement(int speed, int angular_speed) {
    int right_speed = speed + angular_speed;
    int left_speed = speed - angular_speed;

    // Set motor directions
    if (right_speed > 0) {
        digitalWrite(F_M_R, HIGH);
        digitalWrite(R_M_R, LOW);
    } else {
        digitalWrite(F_M_R, LOW);
        digitalWrite(R_M_R, HIGH);
    }

    if (left_speed > 0) {
        digitalWrite(F_M_L, LOW);
        digitalWrite(R_M_L, LOW);
    } else {
        digitalWrite(F_M_L, HIGH);
        digitalWrite(R_M_L, HIGH);
    }


    analogWrite(PWM_F_M_R, abs(right_speed) < 255 ? abs(right_speed) : 255);
    analogWrite(PWM_F_M_L, abs(left_speed) < 255 ? abs(left_speed) : 255);
    analogWrite(PWM_R_M_R, abs(right_speed) < 255 ? abs(right_speed) : 255);
    analogWrite(PWM_R_M_L, abs(left_speed) < 255 ? abs(left_speed) : 255);
}

void setup() {
    Serial.begin(115200); 
    pinMode(F_M_R, OUTPUT);
    pinMode(PWM_F_M_R, OUTPUT);
    pinMode(F_M_L, OUTPUT);
    pinMode(PWM_F_M_L, OUTPUT);
    pinMode(R_M_R, OUTPUT);
    pinMode(PWM_R_M_R, OUTPUT);
    pinMode(R_M_L, OUTPUT);
    pinMode(PWM_R_M_L, OUTPUT);

    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

    delay(1000);

    last_msg_time = millis();  
}

void loop() {
    if (Serial.available() > 0) {
        String incomingData = Serial.readStringUntil('\n');

        if (gpsSerial.available() > 0) {
            char gpsData = gpsSerial.read();
            gps.encode(gpsData);  

            if (gps.location.isUpdated()) {  
                latitude = gps.location.lat(); 
                longitude = gps.location.lng(); 
                altitude = gps.altitude.meters();


                
            }
        }

        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, incomingData);

        if (error) {      
            return;
        }

        const char* command = doc["command"];
        if (command) {
            if (strcmp(command, "identify") == 0) {
                handleIdentification();
                return;
            }
        }

        linear = doc["linear"];
        angular = doc["angular"];

        movement(linear, angular);

        StaticJsonDocument<200> response;
        response["longitude"] = longitude;  
        response["latitude"] = latitude;   
        response["altitude"] = altitude;   

        String output;
        serializeJson(response, output);
        Serial.println(output);

        last_msg_time = millis();
    }

    if (millis() - last_msg_time > msg_timeout) {
        movement(0, 0); 
    }

    delay(10);  
}