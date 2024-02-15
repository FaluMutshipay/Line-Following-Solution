//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot 2023                                     *//
//*                                                      *//
//*  Skeleton Master Code for Use with the               *//
//*  EEEBot_MainboardESP32_Firmware Code                 *//
//*                                                      *//
//*  Nat Dacombe                                         *//
//********************************************************//

// the following code acts as a 'bare bones' template for your own custom master code that works with the firmware code provided
// therefore, the variable names are non-descriptive - you should rename these variables appropriately
// you can either modify this code to be suitable for the project week task, or use the functions as inspiration for your own code

#include <Wire.h>
#include <NewPing.h>
#include <PID_v1.h>

#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
#define TRIGGER_PIN 14
#define ECHO_PIN 27
#define MAX_DISTANCE 150
#define weighted_reference 0.332

const int sensor_max[6] = {4095, 4095, 4095, 3050, 3870, 4095};
const int sensor_min[6] = {2500, 3200, 2900, 2400, 2800, 400};
const int pin[6] = {39, 33, 34, 32, 35, 36};
const int motor_offset = 25;
const int servo_offset = 15;

int P, I, D, PIDvalue, error, previousError, position, previousPosition, i, weightedAverage, sensorSum;
int w0 = 7;
int w1 = 18;
int w2 = 32;
int w3 = 45;
int w4 = 57;
int w5 = 73;
int w_total = 232;
int IR_sensors[6];

float Kp = 50;
float Ki = 0.02;
float Kd = 2.5;
float Pvalue, Ivalue, Dvalue;

uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;
uint8_t Kpfinal, Kifinal, Kdfinal;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup()
{
  Serial.begin(115200);
  Wire.begin();   // join i2c bus (address optional for the master) - on the ESP32 the default I2C pins are 21 (SDA) and 22 (SCL)
}

// three integer values are sent to the slave device
int leftMotor_speed = 250;
int rightMotor_speed = 250;
int servoAngle = 83;

void forwardRight()
{
  leftMotor_speed = 250;
  rightMotor_speed = 175 - motor_offset;
  servoAngle = 93 + servo_offset;

  Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
  Wire.endTransmission();                             
}

void forwardLeft()
{
  leftMotor_speed = 250 - motor_offset;
  rightMotor_speed = 175;
  servoAngle = 93 - servo_offset;

  Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
  Wire.endTransmission();                                    // stop transmitting

}

/*int readline(){
  int setPoint;
  setPoint = ((5000 * IR_sensors[0]) + (4000 * IR_sensors[1]) + (3000 * IR_sensors[2]) + (2000 * IR_sensors[3]) + (1000 * IR_sensors[4]) + IR_sensors[5]) / (4095*6);
  Serial.println(setPoint);
  return setPoint;
}*/

void vehicleControl(){
  // Error calculation for all 6 sensors
  for (i = 0; i < 6; i++){
    weightedAverage += IR_sensors[i]*i*10;
    sensorSum += IR_sensors[i];
  }

  int position = sensorSum > 0 ? weightedAverage/sensorSum : previousPosition;
  //Serial.print(position);
  error = 4095 - position;
  previousError = error;
  previousPosition = position;

  while (IR_sensors[0]>=sensor_max[0] && IR_sensors[1]>=sensor_max[1] && IR_sensors[2]>=sensor_max[2] && IR_sensors[3]>=sensor_max[3] && IR_sensors[4]>=sensor_max[4] && IR_sensors[5]>=sensor_max[5]){ // A case when vehicle is off line
  if (previousError>0){ // Turn left if line was left before
  forwardLeft();
  }
  else{
    forwardRight(); // Else turn right
  }
  
  for (i = 0; i < 6; i++){
    IR_sensors[i] = analogRead(pin[i]);
  }
 
  
  calculatePID(error, previousError);
}}

void calculatePID(int error, int previousError){
  /*uint16_t position;
  position = readline();*/
  Serial.print("ERROR = ");
  Serial.println(error);
  P = error;
  I = I + error;
  D = error-previousError;
  
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);

  Serial.print("PID VALUE = ");
  Serial.println(PIDvalue);
  rightMotor_speed = leftMotor_speed + PIDvalue;
  leftMotor_speed = leftMotor_speed - PIDvalue;

  if (rightMotor_speed >= 250){
    rightMotor_speed = 250;
  }
  if (rightMotor_speed <= -250){
    rightMotor_speed = -250;
  }
  if (leftMotor_speed >= 250){
    leftMotor_speed = 250;
  }
  if (leftMotor_speed <= -250){
    leftMotor_speed = -250;
  }

  Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
  Wire.endTransmission();                                    // stop transmitting
}

void loop()
{
  // two 16-bit integer values are requested from the slave
  int16_t encRight_count = 0;
  int16_t encLeft_count = 0;
  uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 4);  // 4 indicates the number of bytes that are expected
  uint8_t encRight_count16_9 = Wire.read();  // receive bits 16 to 9 of a (one byte)
  uint8_t encRight_count8_1 = Wire.read();   // receive bits 8 to 1 of a (one byte)
  uint8_t encLeft_count16_9 = Wire.read();   // receive bits 16 to 9 of b (one byte)
  uint8_t encLeft_count8_1 = Wire.read();   // receive bits 8 to 1 of b (one byte)

  encRight_count = (encRight_count16_9 << 8) | encRight_count8_1; // combine the two bytes into a 16 bit number
  encLeft_count = (encLeft_count16_9 << 8) | encLeft_count8_1; // combine the two bytes into a 16 bit number

  vehicleControl();
  
  for (i = 0; i < 6; i++){
    IR_sensors[i] = analogRead(pin[i]);
  }

  // print out the value you read:
  Serial.print("Sensor 0 = ");
  Serial.println(IR_sensors[0]);
  delay(200);
  Serial.print("Sensor 1 = ");
  Serial.println(IR_sensors[1]);
  delay(200);
  Serial.print("Sensor 2 = ");
  Serial.println(IR_sensors[2]);
  delay(200);
  Serial.print("Sensor 3 = ");
  Serial.println(IR_sensors[3]);
  delay(200);
  Serial.print("Sensor 4 = ");
  Serial.println(IR_sensors[4]);
  delay(200);
  Serial.print("Sensor 5 = ");
  Serial.println(IR_sensors[5]);
  delay(200);
  
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  /* depending on the microcontroller, the int variable is stored as 32-bits or 16-bits
     if you want to increase the value range, first use a suitable variable type and then modify the code below
     for example; if the variable used to store x and y is 32-bits and you want to use signed values between -2^31 and (2^31)-1
     uncomment the four lines below relating to bits 32-25 and 24-17 for x and y
     for my microcontroller, int is 32-bits hence x and y are AND operated with a 32 bit hexadecimal number - change this if needed

     >> X refers to a shift right operator by X bits
  */
  //Wire.write((byte)((x & 0xFF000000) >> 24)); // bits 32 to 25 of x
  //Wire.write((byte)((x & 0x00FF0000) >> 16)); // bits 24 to 17 of x
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  //Wire.write((byte)((y & 0xFF000000) >> 24)); // bits 32 to 25 of y
  //Wire.write((byte)((y & 0x00FF0000) >> 16)); // bits 24 to 17 of y
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.endTransmission();   // stop transmitting
  delay(100);
}
