#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_PWMServoDriver.h>
#include "MS5837.h"

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire1);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);
MS5837 ms5837;

volatile int sensor_temp[7] = {11, 22, 33, 44, 55,66,-77};
                            // {RL, PH, YW, DS, WS, VS,  CS}0,0
                            // RL, PH, YW CS -32767 to 32767
                            // DS, WS, VS 0 to 255

// Variables for PID speed control
int Setpoint[10]; double Input[10]; double Output[10];
// AB CD 00 00 00 10 00 10 00 10 01 01 00 00 00 00 AA
// AB CD 00 10 00 10 00 10 00 10 01 01 00 00 00 00 BA
// 00 00 SG SG SW SW YW YW HV HV S1 S2 O1 O2 O3 O4 CS
// AB CD FF FE FF FE FF FE FF FE 01 01 00 00 00 00 6e

volatile int receivedIndex = 0; // Index to track the position in the buffer
const int buff_read = 17; // Number of bytes to read
byte receivedBytes[buff_read];
volatile bool newData = false; // Flag to indicate new data is available
bool printSetpoint = true; // Flag to control whether to print Setpoint values

const int buff_write = 14; // Number of bytes to read
byte write_buff[buff_write];
int debg=0;

float rad(float degree){
  return (degree * 71.0) / 4068.0;
}
void printEvent(sensors_event_t* event, Stream* serial) {

  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    serial->print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    serial->print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    serial->print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    serial->print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    serial->print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    serial->print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    serial->print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    serial->print("Unk:");
  }

  serial->print("\tx= ");
  serial->print(x);
  serial->print(" |\ty= ");
  serial->print(y);
  serial->print(" |\tz= ");
  serial->println(z);
}
int cvt_wave_lengh_to_bit(int value,float frq){

  if (value>=1800){
    // Serial.println("over");

    return map(1800,0,1/frq*1000000,0,4096);
  }
  if (value<=1200)  {
    // Serial.println("under");

    return map(1200,0,1/frq*1000000,0,4096);
  }
  // Serial.println("normal");

  return map(value,0,1/frq*1000000,0,4096);

}
struct Motor_output {
  int Motor1;
  int Motor2;
  int Motor3;
  int Motor4;
  int Motor5;
  int Motor6;
  int Motor7;
  int Motor8;
};
Motor_output ForwardKinematic(int Surge, int Sway, int Heave,int Roll,int Pitch, int Yaw, float degree,float radius) {
  // input as percent
  Motor_output output;

  // Simple example logic to generate output values from inputs
  output.Motor1 = ((-Surge/sin(rad(degree)))+(Sway/cos(rad(degree)))+(radius*Yaw/(cos(rad(degree-45)))))/4.0;
  output.Motor2 = ((-Surge/sin(rad(degree)))+(-Sway/cos(rad(degree)))+(-radius*Yaw/(cos(rad(degree-45)))))/4.0;
  output.Motor3 = ((Surge/sin(rad(degree)))+(Sway/cos(rad(degree)))+(-radius*Yaw/(cos(rad(degree-45)))))/4.0;
  output.Motor4 = ((Surge/sin(rad(degree)))+(-Sway/cos(rad(degree)))+(radius*Yaw/(cos(rad(degree-45)))))/4.0;
  output.Motor5 = (Heave+(radius*Roll)-(radius*Pitch))/4;
  output.Motor6 = (Heave-(radius*Roll)-(radius*Pitch))/4;
  output.Motor7 = (Heave+(radius*Roll)+(radius*Pitch))/4;
  output.Motor8 = (Heave-(radius*Roll)+(radius*Pitch))/4;

  // output.Motor1 = ((-Surge/sin(rad(degree)))+(Sway/cos(rad(degree)))+(radius*Yaw/(cos(rad(degree-45)))))/4.0;
  // output.Motor2 = ((-Surge/sin(rad(degree)))+(-Sway/cos(rad(degree)))+(-radius*Yaw/(cos(rad(degree-45)))))/4.0;
  // output.Motor3 = ((Surge/sin(rad(degree)))+(Sway/cos(rad(degree)))+(-radius*Yaw/(cos(rad(degree-45)))))/4.0;
  // output.Motor4 = ((Surge/sin(rad(degree)))+(-Sway/cos(rad(degree)))+(radius*Yaw/(cos(rad(degree-45)))))/4.0;
  // output.Motor5 = (Heave+(radius*Roll)-(radius*Pitch))/4;
  // output.Motor6 = (Heave-(radius*Roll)-(radius*Pitch))/4;
  // output.Motor7 = (Heave+(radius*Roll)+(radius*Pitch))/4;
  // output.Motor8 = (Heave-(radius*Roll)+(radius*Pitch))/4;
  return output;

  // Example 
  // Motor_output Motor = ForwardKinematic(100, 100,100,100,100,100,45.0,0.2);
  // Serial.println("Motor Values:");
  // Serial.println(Motor.Motor1);
  // Serial.println(Motor.Motor2);
  // Serial.println(Motor.Motor3);
  // Serial.println(Motor.Motor4);
  // Serial.println(Motor.Motor5);
  // Serial.println(Motor.Motor6);
  // Serial.println(Motor.Motor7);
  // Serial.println(Motor.Motor8);

  // Motor Values:
  // 5.00
  // -75.71
  // 65.71
  // 5.00
  // 25.00
  // 15.00
  // 35.00
  // 25.00
}
void CONTROL_SENT(Stream* serialPorts[], int count)
{
  for (int i = 0; i < 3; i++) {
    if (sensor_temp[i] < 0) sensor_temp[i] = ((abs(sensor_temp[i]) ^ 0xFFFF) + 0x0001) & 0xFFFF;
    write_buff[2 + i * 2] = (abs(sensor_temp[i]) >> 8) & 0xFF;
    write_buff[3 + i * 2] = abs(sensor_temp[i]) & 0xFF;
    //Serial.print(i);
    // Serial.print(" ");
    // Serial.print(write_buff[2 + i * 2]);
    // Serial.print(" ");
    // Serial.print(write_buff[3 + i * 2]);
    // Serial.print(" ");

  }
  // Serial.println();
  for (int i = 0; i < 3; i++) {
    //if (sensor_temp[7+i] < 0) sensor_temp[7+i] = ((abs(sensor_temp[7+i]) ^ 0xFFFF) + 0x0001) & 0xFFFF;
    //write_buff[2 + i * 2] = (abs(sensor_temp[i]) >> 8) & 0xFF;
    write_buff[8 + i ] = abs(sensor_temp[3+i]) & 0xFF;
    //Serial.print(16 + i );
    // Serial.print(" ");
    // Serial.print(write_buff[16 + i ]);
    // Serial.print(" ");

  }
  
  if (sensor_temp[6] < 0) sensor_temp[6] = ((abs(sensor_temp[6]) ^ 0xFFFF) + 0x0001) & 0xFFFF;
  write_buff[11] = (abs(sensor_temp[6]) >> 8) & 0xFF;
  write_buff[11+1] = abs(sensor_temp[6]) & 0xFF;

  // Serial.println();

  write_buff[buff_write-1] = 0;
  for (int i = 0; i < buff_write-1; i++) write_buff[buff_write-1] += write_buff[i];
  write_buff[buff_write-1] &= 0xFF;
  // Serial.println(write_buff[buff-1]);

  // // Send the encoded values to Serial
  for (int j = 0; j < count; j++) {
    
    for (int i = 0; i < buff_write; i++) {
      //if (write_buff[i] < 0x10) Serial.print("0"); // Add leading zero for single digit hex values
      serialPorts[j]->print(write_buff[i],HEX);
      if (i < buff_write - 1) serialPorts[j]->print(" "); // Separate values with a space
    }
  }
  for (int j = 0; j < count; j++){
    serialPorts[j]->println(); // Newline at the end
  }


  debg+=1;
  //Serial.println(debg);
}
void processBuffer(byte buffer[],Stream &serialPort) {
  // Convert buffer Setpoint to values in the Setpoint array
  // 00 00 SG SG SW SW YW YW HV HV S1 S2 O1 O2 O3 O4 CS

  Setpoint[0] = buffer[2] << 8 | buffer[3];
  Setpoint[1] = buffer[4] << 8 | buffer[5];
  Setpoint[2] = buffer[6] << 8 | buffer[7];
  Setpoint[3] = buffer[8] << 8 | buffer[9];
  Setpoint[4] = buffer[10];
  Setpoint[5] = buffer[11];
  Setpoint[6] = buffer[12];
  Setpoint[7] = buffer[13];
  Setpoint[8] = buffer[14];
  Setpoint[9] = buffer[15];


  // Check the most significant bit (MSB) to adjust the number to negative if needed
  if (buffer[2] & 0x80) Setpoint[0] = ((Setpoint[0] ^ 0xFFFF) + 0x0001) * -1;
  if (buffer[4] & 0x80) Setpoint[1] = ((Setpoint[1] ^ 0xFFFF) + 0x0001) * -1;
  if (buffer[6] & 0x80) Setpoint[2] = ((Setpoint[2] ^ 0xFFFF) + 0x0001) * -1;
  if (buffer[8] & 0x80) Setpoint[3] = ((Setpoint[3] ^ 0xFFFF) + 0x0001) * -1;
  
  // Print the Setpoint values via Serial if the flag is set
  if (printSetpoint) {
    serialPort.print("Setpoint: ");
    for (int i = 0; i < 10; i++) {
      serialPort.print(Setpoint[i]);
      if (i < 9) serialPort.print(", ");
    }
    serialPort.println();
  }
}
void handleSerialRx(Stream &serialPort,Stream &serialPort_debug) {
  // Check if there is data available on Serial
  if (!serialPort.available()) return;

  byte incomingByte = serialPort.read(); // Read one byte
  // Serial.println(incomingByte,HEX);

  if (receivedIndex == 0 && incomingByte != 0xAB) return; 
  if (receivedIndex == 1 && incomingByte != 0xCD) {
    receivedIndex = 0; // If the second byte is not 0xCD, reset index
    return;
  }
  receivedBytes[receivedIndex++] = incomingByte;

  // If the buffer is full
  if (receivedIndex >= buff_read) {
    byte checksum = 0;

    // Calculate checksum by summing bytes from position 0 to 9
    for (int i = 0; i < buff_read-1; i++) checksum += receivedBytes[i];
    checksum &= 0xFF; // Keep only the last 8 bits of the checksum
    
    // Verify that the calculated checksum matches the checksum in the buffer
    if (checksum == receivedBytes[buff_read-1]) {
      newData = false;
      // Serial.println("debbbb");
      processBuffer((byte*)receivedBytes, serialPort_debug);
      // Reset the index
      receivedIndex = 0;
    } else {
      // Reset the index if checksum does not match
      receivedIndex = 0;
    }
  }
}

// void CONTROL_RECV(){

//   for( ;; ) {
//     handleSerialRx(Serial3);

//   }
// }
const float pwm_frequency=50.0f;

int surge = 0;
int sway = 0;
int yaw = 0;
int heave = 0;
int middle=1500;
int highest = 2000;
int lowset = 1000;

void setup() {
  //Adafruit_BNO055 bno = *((Adafruit_BNO055 *)pvParameters);
  
  bno.begin();
  // int8_t temp = bno.getTemp();
  
  bno.setExtCrystalUse(false);

  pwm.begin();

  
  while (!ms5837.init(Wire1)) {
    Serial.println("MS5837 not found on Wire1");
    delay(1000);
  }

  ms5837.setModel(MS5837::MS5837_30BA); // Bar30
  ms5837.setFluidDensity(997);          // fresh water

  pwm.setOscillatorFrequency(26000000);
  pwm.setPWMFreq(pwm_frequency);

  pwm.setPWM(0, 0,cvt_wave_lengh_to_bit(middle,pwm_frequency));
  pwm.setPWM(1, 0,cvt_wave_lengh_to_bit(middle,pwm_frequency));
  pwm.setPWM(2, 0,cvt_wave_lengh_to_bit(middle,pwm_frequency));
  pwm.setPWM(3, 0,cvt_wave_lengh_to_bit(middle,pwm_frequency));
  pwm.setPWM(4, 0,cvt_wave_lengh_to_bit(middle,pwm_frequency));
  pwm.setPWM(5, 0,cvt_wave_lengh_to_bit(middle,pwm_frequency));
  pwm.setPWM(6, 0,cvt_wave_lengh_to_bit(middle,pwm_frequency));
  pwm.setPWM(7, 0,cvt_wave_lengh_to_bit(middle,pwm_frequency));
  pwm.setPWM(8, 0,cvt_wave_lengh_to_bit(middle,pwm_frequency));
  pwm.setPWM(9, 0,cvt_wave_lengh_to_bit(middle,pwm_frequency));

  // Serial.begin(115200);
  Serial3.begin(115200);

  Serial3.attachCts(19);
  Serial3.attachRts(2);

  write_buff[0] = 0xDC; 
  write_buff[1] = 0xBA;
}

void loop() {
  
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  // printEvent(&orientationData, &Serial);

  sensor_temp[0] = orientationData.orientation.z;
  sensor_temp[1] = orientationData.orientation.y;
  sensor_temp[2] = orientationData.orientation.x;

  // printEvent(&angVelocityData, &Serial);
  // printEvent(&linearAccelData, &Serial);
  // printEvent(&magnetometerData, &Serial);
  // printEvent(&accelerometerData, &Serial);
  // printEvent(&gravityData, &Serial);

  // int8_t boardTemp = bno.getTemp();
  // Serial.println();
  // Serial.print(F("temperature: "));
  // Serial.println(boardTemp);

  // uint8_t system, gyro, accel, mag = 0;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.println();
  // Serial.print("Calibration: Sys=");
  // Serial.print(system);
  // Serial.print(" Gyro=");
  // Serial.print(gyro);
  // Serial.print(" Accel=");
  // Serial.print(accel);
  // Serial.print(" Mag=");
  // Serial.println(mag);

  // Serial.println("--");


  // AB CD 00 00 00 10 00 10 00 10 01 01 00 00 00 00 AA
  // AB CD 00 10 00 10 00 10 00 10 01 01 00 00 00 00 BA
  // 00 00 SG SG SW SW YW YW HV HV S1 S2 O1 O2 O3 O4 CS
  // AB CD FF FE FF FE FF FE FF FE 01 01 00 00 00 00 6e
  surge = Setpoint[0];
  sway = Setpoint[1];
  yaw = Setpoint[2];
  heave = Setpoint[3];
  // surge = 100;
  // sway = 0;
  // yaw = 0;
  // heave = 0;
  

  Motor_output Motor = ForwardKinematic(surge, sway,heave,0,0,yaw,45.0,1);
  // pwm.setPWM(2, 0,cvt_wave_lengh_to_bit(1450,pwm_frequency));

  pwm.setPWM(4, 0,cvt_wave_lengh_to_bit(middle+(Motor.Motor1*5),pwm_frequency));
  pwm.setPWM(0, 0,cvt_wave_lengh_to_bit(middle-(Motor.Motor2*5),pwm_frequency));
  pwm.setPWM(7, 0,cvt_wave_lengh_to_bit(middle-(Motor.Motor3*5),pwm_frequency));
  pwm.setPWM(3, 0,cvt_wave_lengh_to_bit(middle+(Motor.Motor4*5),pwm_frequency));
  pwm.setPWM(5, 0,cvt_wave_lengh_to_bit(middle+(Motor.Motor5*5),pwm_frequency));
  pwm.setPWM(1, 0,cvt_wave_lengh_to_bit(middle-(Motor.Motor6*5),pwm_frequency));
  pwm.setPWM(6, 0,cvt_wave_lengh_to_bit(middle-(Motor.Motor7*5),pwm_frequency));
  pwm.setPWM(2, 0,cvt_wave_lengh_to_bit(middle+(Motor.Motor8*5),pwm_frequency));

  // Serial3.print("Motor Values:");
  // Serial3.print(Motor.Motor1);
  // Serial3.print(" | ");
  // Serial3.print(Motor.Motor2);
  // Serial3.print(" | ");
  // Serial3.print(Motor.Motor3);
  // Serial3.print(" | ");
  // Serial3.print(Motor.Motor4);
  // Serial3.print(" | ");
  // Serial3.print(Motor.Motor5);
  // Serial3.print(" | ");
  // Serial3.print(Motor.Motor6);
  // Serial3.print(" | ");
  // Serial3.print(Motor.Motor7);
  // Serial3.print(" | ");
  // Serial3.println(Motor.Motor8);

  // Serial.print("Motor Values:");
  // Serial.print(Motor.Motor1);
  // Serial.print(" | ");
  // Serial.print(Motor.Motor2);
  // Serial.print(" | ");
  // Serial.print(Motor.Motor3);
  // Serial.print(" | ");
  // Serial.print(Motor.Motor4);
  // Serial.print(" | ");
  // Serial.print(Motor.Motor5);
  // Serial.print(" | ");
  // Serial.print(Motor.Motor6);
  // Serial.print(" | ");
  // Serial.print(Motor.Motor7);
  // Serial.print(" | ");
  // Serial.println(Motor.Motor8);

  ms5837.read();

  sensor_temp[3] = ms5837.depth()*100; // Depth in cm
  //Serial.println(debg);
  // vTaskDelay(20);
  Stream* ports[] = { &Serial3};

  CONTROL_SENT(ports, 1);

  handleSerialRx(Serial3,Serial);
  // Serial.print("Setpoint:");
  // Serial.print(surge);
  // Serial.print(" | ");
  // Serial.print(sway); 
  // Serial.print(" | ");
  // Serial.print(yaw);
  // Serial.print(" | ");
  // Serial.println(heave);

  delay(10);

}
