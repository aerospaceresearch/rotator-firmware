//rotator.ino - DGSN Satellite-Antenna-Rotator, Firmware for Arduino
//Copyright (c) 2018 Andreas HORNIG, AerospaceResearch.net, DH0RN
//Released under the MIT License.
//For more information please visit https://www.aerospaceresearch.net
//For source code please visit https://github.com/aerospaceresearch/rotator-firmware
//Control commands are based on RotControl ROTCTL, and modified parameters.


// libraries and dependencies
#define SerialPort Serial
#include "ctype.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>


/* Assigning a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
/* Assigning a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);



// Assigning Global variables
String cmd = ""; 
String input_line = "";

bool motor1_az_h1, motor1_az_h2;
bool motor2_el_h1, motor2_el_h2;
int speed1_az, speed2_el;



/* Setup for the following Rotators */
// * Rotator Atlas - light antennas < 0.5 kg (future)
// * Rotator Behemoth - medium antennas < 1.0 kg (current baseline)
// * Rotator Ceres > 1.0 kg (future)
/* based on Arduino Pro Micro */


// motor azimuth
#define pin1 4
#define pin2 5
#define enable_motor_el 6 // only there pwm

// motor elevation
#define pin3 8
#define pin4 9
#define enable_motor_az 10 // only there pwm


float elevation = 0.0;
float azimuth = 0.0;
float heading_mag = 0.0;
float roll = 0.0;
float pitch = 0.0;

float declination_magnetic = 0.0;

// target
float elevation_target = 0.0;
float azimuth_target = 0.0;
float azimuth_target_short = 0.0;
float elevation_dif = 0.0;
float azimuth_dif = 0.0;

int drive = 1;

float pi = 3.1415;


float azimuth_park = 0.0;
float elevation_park = 0.0;
float azimuth_windup = 0.0;
float azimuth_windup_limit = 400.0;

float azimuth_b4 = 0.0;
float azimuth_now = 0.0;

int rot_reverse = 0;



String model_name = "DGSN Satellite-Antenna-Rotator, Firmware for Arduino";
String model_version = "0.01";
String model_date = "2018-12-23";
String model_projectpage = "www.AerospaceResearch.net";
String model_source = "https://github.com/aerospaceresearch/rotator-firmware";


void displayRotatorDetails(void)
{
  Serial.println("------------------------------------");
  Serial.print  ("Rotator-Firmware Model:"); Serial.println(model_name);
  Serial.print  ("Model Ver:    "); Serial.println(model_version);
  Serial.print  ("Model Date:   "); Serial.println(model_date);
  Serial.print  ("Model Project:"); Serial.println(model_projectpage);
  Serial.print  ("Min Value:    "); Serial.println(model_source);
  Serial.println("------------------------------------");
  Serial.println("");
}




void setup() {
  // put your setup code here, to run once:
  SerialPort.begin(9600);

  // for the Arduino Pro Micro layout
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(enable_motor_el, OUTPUT);
  
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);
  pinMode(enable_motor_az, OUTPUT);
  
  // Set initial rotation direction. it is stop!
  motor1_az_h1 = LOW;
  motor1_az_h2 = LOW;
  motor2_el_h1 = LOW;
  motor2_el_h2 = LOW;




  #ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
  #endif

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* Sensor not detected ... check your connections! */
    Serial.println("Sensor (LSM303) not detected ... check your connections!");
    //while(1);
  }
  

  /* Enable auto-gain */
  mag.enableAutoRange(true);

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* Sensor not detected ... check your connections! */
    Serial.println("Sensor (LSM303) not detected ... check your connections!");
    //while(1);
  }
  
}



void windup(void){
  // helps protecting the antenna to turn to much and windup the cable.
  // BUT it is pretty ugly ugly code! :D

  if (azimuth > 45 - 5 and azimuth < 45 + 5 and azimuth_b4 != 45) {
    //Serial.println(45);
    //Serial.println(azimuth_b4);
    if (azimuth_b4 != 0){

      if (45 - azimuth_b4 <= -270){
        azimuth_windup += 90.0;
      } else {
        azimuth_windup += 45 - azimuth_b4;
      }
    }
    azimuth_b4 = 45;
  }

  if (azimuth > 135 - 5 and azimuth < 135 + 5 and azimuth_b4 != 135) {
    //Serial.println(135);
    //Serial.println(azimuth_b4);
    azimuth_windup += 135 - azimuth_b4;
    
    azimuth_b4 = 135;
  }


  if (azimuth > 225 - 5 and azimuth < 225 + 5 and azimuth_b4 != 225) {
    //Serial.println(225);
    //Serial.println(azimuth_b4);
    azimuth_windup += 225 - azimuth_b4;
    
    azimuth_b4 = 225;
  }

  if (azimuth > 315 - 5 and azimuth < 315 + 5 and azimuth_b4 != 315) {
    //Serial.println(315);
    //Serial.println(azimuth_b4);
    if (azimuth_b4 != 0){

      if (315 - azimuth_b4 >= 270){
        azimuth_windup += -90.0;
      } else {
        azimuth_windup += 315 - azimuth_b4;
      }
    }
    
    azimuth_b4 = 315;
  }


  // now it is over winding
  if (azimuth_windup >= azimuth_windup_limit){
    rot_reverse = 1;
    drive == 1;
    motor1_az_h1 = HIGH;
    motor1_az_h2 = LOW;

    speed1_az = 255;
    //Serial.println("stop and reverse");
  }

  // now it is under winding
  if (azimuth_windup <= -azimuth_windup_limit){
    rot_reverse = 1;
    drive == 1;
    motor1_az_h1 = LOW;
    motor1_az_h2 = HIGH;

    speed1_az = 255;
    //Serial.println("stop and reverse");
  }

  if (azimuth_windup >= 0.0 and azimuth_windup <= 45 and rot_reverse == 1){
    rot_reverse = 0;
    drive == 0;
    motor1_az_h1 = LOW;
    motor1_az_h2 = LOW;
    //Serial.println("now ok");
  }
  //Serial.println(azimuth_windup);
  
}


void processCommands(void) {
  // processing the cmd that was sent via Serial as a String.
  
  while (SerialPort.available() > 0) {
    char chr = SerialPort.read();                 //Read a single character from the serial buffer

    if (chr == 10) {                              //Line feed received
      interpreting_commands(input_line);                                //Process Easycomm commands
      input_line = "";
      
    } else if (chr == 13) {                       //Carriage return received
      //interpreting_commands(input_line);        //Process user commands
      input_line = "";
      
    } else {
      input_line += chr;
    }
  }
}

void get_pos(void){
  Serial.print(azimuth);Serial.print(";");Serial.print(elevation);Serial.print(";");Serial.print(heading_mag);Serial.print(";");
      Serial.print(roll);Serial.print(";");Serial.print(pitch);Serial.print(";");Serial.print(azimuth_windup);Serial.print(";");Serial.print(rot_reverse);Serial.println(";");
}




void interpreting_commands(String input_line){
  // interpreting the command string into the argument and value parts.
  
  cmd = input_line;

  int word_1, word_2, word_3;
  String input;
  
  word_1 = cmd.indexOf(' ');
  word_2 = cmd.indexOf(' ', word_1 + 1);
  word_3 = cmd.indexOf(' ', word_2 + 1);

  if (cmd.length() > 0 or word_1 > 0) {

    input = input_line.substring(0, word_1);

    if (input == "set_pos" or input == "P") {
      // P, set_pos 'Azimuth' 'Elevation'

      // setting or resetting the normal mode for set_position
      drive = 0;

      if (word_2 >= 0 and word_2 < cmd.length()-1) {
        input = input_line.substring(word_1, word_2);
        azimuth_target = input.toFloat();
        input = input_line.substring(word_2, word_3); 
        elevation_target = input.toFloat();

        //Serial.print(azimuth_target); Serial.print(";");Serial.println(elevation_target);
              
      } else {
        Serial.println("not enough arguments. 2 arguments needed!");
        
      }
      
    } else if (input == "get_pos" or input == "p") {
      //Serial.println("print position");
      get_pos();

    } else if (input == "move" or input == "M") {
      // M, move 'Direction' 'Speed'
      
      // Move the rotator in a specific direction at the given rate.
      // Values are integers where Direction is defined as
      // 2 = Up, 4 = Down, 8 = Left, and 16 = Right.
      // Speed is an integer between 1 and 100.
      // Not all backends that implement the move command use the Speed value. At this time only the gs232a utilizes the Speed parameter.
      // (Altered for testing. Previous commands needs to be implemented
      
      // signaling the move mode that overwrites the normal set_position mode.
      drive = 1;


      if (word_1 >= 0) {
        input = input_line.substring(word_1+1, word_2);

        String direction_speed = input_line.substring(word_2+1, word_3);

        if (input == "left") {
          // left turn
          motor1_az_h1 = HIGH;
          motor1_az_h2 = LOW;

          if (word_2 > -1 and word_2 < input_line.length()-1) {
            speed1_az = direction_speed.toInt();
          } else {
            speed1_az = 255;
          }
          
        } else if (input == "right") {
          // right turn
          motor1_az_h1 = LOW;
          motor1_az_h2 = HIGH;

          if (word_2 > -1 and word_2 < input_line.length()-1) {
            speed1_az = direction_speed.toInt();
          } else {
            speed1_az = 255;
          }
          
        } else if (input == "up") {
          // left turn
          motor2_el_h1 = HIGH;
          motor2_el_h2 = LOW;

          if (word_2 > -1 and word_2 < input_line.length()-1) {
            speed2_el = direction_speed.toInt();
          } else {
            speed2_el = 255;
          }
          
        } else if (input == "down") {
          // down turn
          motor2_el_h1 = LOW;
          motor2_el_h2 = HIGH;

          if (word_2 > -1 and word_2 < input_line.length()-1) {
            speed2_el = direction_speed.toInt();
          } else {
            speed2_el = 255;
          }
        }
              
      } else {
        Serial.println("not enough arguments. 2 arguments needed!");
        
      }

    } else if (input == "stop" or input == "S") {
      //Serial.println("stop rotor");

      drive = 1;

      // stop motor1
      motor1_az_h1 = LOW;
      motor1_az_h2 = LOW;

      // stop motor2
      motor2_el_h1 = LOW;
      motor2_el_h2 = LOW;
    
    } else if (input == "park" or input == "K") {
      Serial.println("parking");
      azimuth_target = azimuth_park;
      elevation_target = elevation_park;
      drive = 0;
    
    } else if (input == "set_conf" or input == "C") {

      if (word_1 >= 0) {
        input = input_line.substring(word_1+1, word_2);

        String value = input_line.substring(word_2+1, word_3);
        
        if (input == "declination_magnetic" and word_2 > -1 and word_2 < input_line.length()-1 or input == "dec_mag" and word_2 > -1 and word_2 < input_line.length()-1) {
          declination_magnetic = value.toFloat();
        }

        Serial.print("dec_mag: ");Serial.println(declination_magnetic);
      }

    } else if (input == "reset" or input == "R") {
      Serial.println("reset");
    
    } else if (input == "get_info" or input == "_") {
      //Serial.println("get_info");
      /* Display some basic information */
      displayRotatorDetails();
    
    } else if (input == "send_cmd" or input == "w") {
      Serial.println("send_cmd");
    }
    
  }
}






void moveit(void) {
  // moving the rotator motors according to the differences of elevation and azimuth.
  // the code tries to minimize the differences between the current positions and their wanted target positions.

  // both values selected because they just worked. further improvement needed. :)
  float angle_max = 0.9;
  float angle_stop = 2.0;
  

  // elevation difference
  elevation_dif = elevation_target - elevation;
  if (elevation_dif < -angle_max and drive == 0){
    motor2_el_h1 = LOW;
    motor2_el_h2 = HIGH;
    speed2_el = 255;
  }

  if (elevation_dif > angle_max and drive == 0){
    motor2_el_h1 = HIGH;
    motor2_el_h2 = LOW;
    speed2_el = 255;
  }

  if (elevation_dif > -angle_stop and elevation_dif < angle_stop  and drive == 0){
    motor2_el_h1 = LOW;
    motor2_el_h2 = LOW;
  }



  // azimuth difference
  azimuth_dif = azimuth_target - azimuth;
  // using the shortest turning direction
  if (abs(azimuth_target - azimuth) > 180.0){
    azimuth_dif = -azimuth_dif;
  }
  
  if (azimuth_dif < -angle_max and drive == 0 and rot_reverse == 0){
    motor1_az_h1 = HIGH;
    motor1_az_h2 = LOW;
    speed1_az = 255;
  }

  if (azimuth_dif > angle_max and drive == 0 and rot_reverse == 0){
    motor1_az_h1 = LOW;
    motor1_az_h2 = HIGH;
    speed1_az = 255;
  }

  if (azimuth_dif > -angle_stop and azimuth_dif < angle_stop  and drive == 0 and rot_reverse == 0){
    motor1_az_h1 = LOW;
    motor1_az_h2 = LOW;
  }
  


  // controlling the two motors with their (new) sets of directions and speeds
  digitalWrite(pin1, motor1_az_h1);
  digitalWrite(pin2, motor1_az_h2);
  analogWrite(enable_motor_el, speed1_az);

  digitalWrite(pin3, motor2_el_h1);
  digitalWrite(pin4, motor2_el_h2);
  analogWrite(enable_motor_az, speed2_el);
  
  delay(200);
  
}


float pitch_and_roll(float xa, float ya, float za, float xm, float ym, float zm){
  // coordinate transform to get the magetic vector.
  // for that the roll and pitch angles needs to be determined.
  // the magnatic vector is than always parallel to the surface.
  
  // https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout?view=all
  //float xa = x_acc;
  //float ya = y_acc;
  //float za = z_acc;
  float norma = sqrt(xa*xa + ya*ya + za*za);

  xa = xa / norma;
  ya = ya / norma;
  za = za / norma;

  ya = -ya;
  za = -za;


  //float xm = event_mag.magnetic.x;
  //float ym = event_mag.magnetic.y;
  //float zm = event_mag.magnetic.z;
  float normm = sqrt(xm*xm + ym*ym + zm*zm);

  xm = xm / normm;
  ym = ym / normm;
  zm = zm / normm;

  ym = -ym;
  zm = -zm;


  pitch = asin(xa);
  roll = -asin(ya);

  float xmc = xm * cos(pitch) + zm * sin(pitch);
  float ymc = ym * cos(roll) - zm * sin(roll);

  heading_mag = atan2(-ymc, xmc) * 180/pi;

  if (heading_mag < 0){
    heading_mag += 360;
  }

  return heading_mag;
}



void sensor(void){
  /* Get a new sensor event for acceleration */
  sensors_event_t event_acc;
  accel.getEvent(&event_acc);

  float x_acc = event_acc.acceleration.x;
  float y_acc = event_acc.acceleration.y;
  float z_acc = event_acc.acceleration.z;

  elevation = atan(x_acc / z_acc) * 180 / pi;
  
  /* Get a new sensor event for magnetics */
  sensors_event_t event_mag;
  mag.getEvent(&event_mag);

  azimuth = pitch_and_roll(x_acc, y_acc, z_acc, event_mag.magnetic.x, event_mag.magnetic.y, event_mag.magnetic.z) - declination_magnetic;
  
}



void loop() {
  // put your main code here, to run repeatedly:

  processCommands();
  moveit();
  
  if(accel.begin())
  // allowing to test other functions even when the sensor is not attached.
  {
    sensor();
  }
  windup();

}
