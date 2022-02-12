#include <QMC5883LCompass.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>


#define echo_front 5
#define trig_front 6

#define echo_left 11
#define trig_left 12

#define echo_right 14
#define trig_right 15

#define in1 7
#define in2 8
#define in3 9
#define in4 10
#define HIGH 1
#define LOW 0

//in1 and  in2 left
//in3 and in4 right

long duration_front;
int distance_front;


long duration_left;
int distance_left;


long duration_right;
int distance_right;
int safe_distance = 10;

byte left = 0;
byte front = 1;
byte right = 2;

int is_clear[3] = {0, 0, 0};

float lat = 41.6124, lon = 14.2766; // create variable for latitude and longitude object
SoftwareSerial gpsSerial(3, 4); //rx,tx
TinyGPS gps; // create gps object


QMC5883LCompass compass;
int inputByte = 0;
byte azimuth;
char myArray[3];

void setup() {
  Serial.begin(9600);
  Serial.println("The GPS Received Signal:");
  gpsSerial.begin(9600);
  Wire.begin();

  proximity_sensors_init();
  engine_init();
  compass.init();// Initialize the serial port.

}

void proximity_sensors_init() {
  // front sensor init
  pinMode(trig_front, OUTPUT);
  pinMode(echo_front, INPUT);

  //left sensor init
  pinMode(trig_left, OUTPUT);
  pinMode(echo_left, INPUT);

  //right sensor init
  pinMode(trig_right, OUTPUT);
  pinMode(echo_right, INPUT);
  Serial.println("Sensors initalized");

}

void engine_init() {

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Serial.println("engine initialized");
}

void loop() {
  go_forward();
  gps_try();
  front_sensor();
  bluetooth_try();
}

void front_sensor() {

  // Clears the trigPin condition
  digitalWrite(trig_front, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trig_front, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_front, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration_front = pulseIn(echo_front, HIGH);
  // Calculating the distance
  distance_front = (duration_front - 10) * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance_front);
  Serial.println(" cm");
}


void right_sensor() {

  // Clears the trigPin condition
  digitalWrite(trig_right, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trig_right, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_right, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration_right = pulseIn(echo_right, HIGH);
  // Calculating the distance
  distance_right = (duration_right - 10) * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance_right);
  Serial.println(" cm");
}


void left_sensor() {

  // Clears the trigPin condition
  digitalWrite(trig_left, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trig_left, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_left, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration_left = pulseIn(echo_left, HIGH);
  // Calculating the distance
  distance_left = (duration_left - 10) * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance_left);
  Serial.println(" cm");
}
void gps_try() {

  while (gpsSerial.available()) { // check for gps data
    if (gps.encode(gpsSerial.read())) // encode gps data
    {
      gps.f_get_position(&lat, &lon); // get latitude and longitude
      // display position
      Serial.print("latitudine: ");
      Serial.println(lat);

      Serial.print("longitudine: ");
      Serial.println(lon);
    }


  }
}
void bluetooth_try() {

  while (Serial.available() > 0) {
    inputByte = Serial.read();
    Serial.println(inputByte);
    if (inputByte == 1) {
      digitalWrite(13, HIGH);
    }
    else if (inputByte == 0) {
      digitalWrite(13, LOW);
    }
  }
}
void compass_try() {

  compass.read();
  azimuth = compass.getAzimuth();
  compass.getDirection(myArray, azimuth);

  Serial.print(myArray[0]);
  Serial.print(myArray[1]);
  Serial.print(myArray[2]);
  Serial.println();

  delay(250);
}

//--------------------------OBJECTIVE-------------------------------------------
void set_base() {}
void acquire_objective() {}
void to_objective() {}
//------------------------------------------------------------------------------

//--------------------------AVOIDANCE-------------------------------------------
void avoid() {}
void check_lefft() {}
void check_right() {}
void check_front() {}
//-------------------------------------------------------------------------------

//-----------------------MOVEMENT------------------------------------------------
void turn_left() {
  left_wheel_forward();
  right_wheel_stop();
  Serial.println("LEFT");
}
void turn_right() {
  left_wheel_stop();
  right_wheel_forward();
  Serial.println("RIGHT");
}
void left_wheel_forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void right_wheel_forward() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void left_wheel_backward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void right_wheel_backward() {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}


void left_wheel_stop() {

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}


void right_wheel_stop() {

  digitalWrite(in3,  LOW);
  digitalWrite(in4, LOW);
}

void go_forward() {
  left_wheel_forward();
  right_wheel_forward();
  Serial.println("FORWARD");
}
void go_backward() {
  left_wheel_backward();
  right_wheel_backward();
  Serial.println("BACKWARD");
}
//--------------------------------------------------------------------------------------------------------------------
