

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "BluetoothSerial.h"

//motor_output_variables
#define MOTOR1_PIN 33 // Top Right motor (motor 1)
#define MOTOR2_PIN 26  // Top Left motor (motor 2)
#define MOTOR3_PIN 19  // Bottom Left motor (motor 3)
#define MOTOR4_PIN 32  // Bottom Right motor (motor 4)
#define MOTOR1_CHANNEL 0
#define MOTOR2_CHANNEL 1
#define MOTOR3_CHANNEL 2
#define MOTOR4_CHANNEL 3
#define MOTOR_PWM_FREQ 400
#define MOTOR_PWM_RESOLUTION 12 // 12-bit resolution for duty cycle (0~4096)
float m1Out = 0, m2Out = 0, m3Out = 0, m4Out = 0;

//ppm_variables
#define PPM_PIN 34
#define PPM_CHANNELS 8
volatile unsigned long ppm_lastInterruptTime = 0;
volatile int ppm_channelIndex = 0;
volatile int ppm_vals[PPM_CHANNELS];

//target_input_variable
int rollInput = 0;
int throttleInput = 0;
int pitchInput = 0;
int yawInput = 0;
int armInput = 0;

//flight_status_variable
bool armed;
float roll;
float pitch;
float rollRate;
float pitchRate;
float yawRate;

//time_variable
float angle_loop_time = 0.004;
float rate_loop_time = 0.002;
unsigned long rate_lastLoopTime = 0;
unsigned long angle_lastLoopTime = 0;
unsigned long CurrentTime = 0;

//pid variable
float roll_error_rate = 0; float roll_pr_error_rate = 0;
float pitch_error_rate = 0; float pitch_pr_error_rate = 0;
float yaw_error_rate = 0; float yaw_pr_error_rate = 0;
float kp_roll_rate = 0.0 ; float kp_pitch_rate = 0.00 ; float kp_yaw_rate = 0;
float ki_roll_rate = 0.0 ; float ki_pitch_rate = 0.00 ; float ki_yaw_rate = 0;
float kd_roll_rate = 0.0 ; float kd_pitch_rate = 0.00 ; float kd_yaw_rate = 0;
float pitch_pid_output ; float roll_pid_output ; float yaw_pid_output ;
float rollIntegral_rate = 0;float pitchIntegral_rate = 0;float yawIntegral_rate =0;

float roll_error_angle = 0; float roll_pr_error_angle = 0;
float pitch_error_angle = 0; float pitch_pr_error_angle = 0;
float yaw_error_angle = 0; float yaw_pr_error_angle = 0;
float kp_roll_angle = 0.0 ; float kp_pitch_angle = 0.00 ; float kp_yaw_angle = 0;
float ki_roll_angle = 0.0 ; float ki_pitch_angle = 0.00 ; float ki_yaw_angle = 0;
float kd_roll_angle = 0.0 ; float kd_pitch_angle = 0.00 ; float kd_yaw_angle = 0;
float pitch_pid_rate ; float roll_pid_rate ; float yaw_pid_rate ;
float rollIntegral_angle = 0;float pitchIntegral_angle = 0;float yawIntegral_angle =0;

//pid tunning variable
String inputString = "";
String send = "";

//mpu calibration variables
float mpu_offsets[8] = {0,0,0,0,0,0,0,0};
int nsamples = 5000;
bool mpu_calib_flag = true;


//debugging variables
int count = 0;


MPU6050 mpu;
BluetoothSerial SerialBT;

void IRAM_ATTR ppm_isr() {
  unsigned long now = micros();
  unsigned long diff = now - ppm_lastInterruptTime;
  if (diff > 3000) {
    // Sync pulse detected, reset channel index
    ppm_channelIndex = 0;
  } else {
    if (ppm_channelIndex < PPM_CHANNELS) {
      ppm_vals[ppm_channelIndex++] = diff;
    }
  }
  ppm_lastInterruptTime = now;
}
void setupMotors() {
  ledcSetup(MOTOR1_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR1_PIN, MOTOR1_CHANNEL);
  ledcSetup(MOTOR2_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR2_PIN, MOTOR2_CHANNEL);
  ledcSetup(MOTOR3_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR3_PIN, MOTOR3_CHANNEL);
  ledcSetup(MOTOR4_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR4_PIN, MOTOR4_CHANNEL);
  // Initialize all motors to off
  ledcWrite(MOTOR1_CHANNEL, 1024);
  ledcWrite(MOTOR2_CHANNEL, 1024);
  ledcWrite(MOTOR3_CHANNEL, 1024);
  ledcWrite(MOTOR4_CHANNEL, 1024);
}
void setupPPM() {
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppm_isr, RISING);
}
void readPPM() {
  // Copy volatile ppm values
  noInterrupts();
  int ppmCopy[PPM_CHANNELS];
  for (int i = 0; i < PPM_CHANNELS; i++) {
    ppmCopy[i] = ppm_vals[i];
  }
  interrupts();
  //channels: 0=roll,1=throttle,2=pitch,3=yaw,5=arm
  rollInput     = ppmCopy[0];
  throttleInput = ppmCopy[2];
  pitchInput    = ppmCopy[1];
  yawInput      = ppmCopy[3];
  armInput      = ppmCopy[5];
  rollInput = map(rollInput,1000,2000,-30,30);
  pitchInput = map(pitchInput,1000,2000,-30,30);
  yawInput = map(yawInput,1000,2000,-8,8);
  // Arming switch logic
  if (armInput > 1500) {
    armed = true;
  } else {
    armed = false;
  }
}
void mixMotorOutputs() {
  // rollInput-=1500;
  // pitchInput-=1500;
  // yawInput-=1500;
  m1Out = throttleInput+pitch_pid_output-roll_pid_output-yaw_pid_rate;
  m2Out = throttleInput+pitch_pid_output+roll_pid_output+yaw_pid_rate;
  m3Out = throttleInput-pitch_pid_output+roll_pid_output-yaw_pid_rate;
  m4Out = throttleInput-pitch_pid_output-roll_pid_output+yaw_pid_rate;
  m1Out = constrain(m1Out, 1000, 2000);
  m2Out = constrain(m2Out, 1000, 2000);
  m3Out = constrain(m3Out, 1000, 2000);
  m4Out = constrain(m4Out, 1000, 2000);
}
void writeMotors() {
  int duty2 = max(1650,(int)(m2Out*1.6384));
  int duty3 = max(1650,(int)(m3Out*1.6384));
  int duty4 = max(1650,(int)(m4Out*1.6384));
  int duty1 = max(1650,(int)(m1Out*1.6384));
  if (armed) {
    ledcWrite(MOTOR1_CHANNEL, duty1);
    ledcWrite(MOTOR2_CHANNEL, duty2);
    ledcWrite(MOTOR3_CHANNEL, duty3);
    ledcWrite(MOTOR4_CHANNEL, duty4);
  }
  else{
    ledcWrite(MOTOR1_CHANNEL, 1638);
    ledcWrite(MOTOR2_CHANNEL, 1638);
    ledcWrite(MOTOR3_CHANNEL, 1638);
    ledcWrite(MOTOR4_CHANNEL, 1638);
  }
}
class Kalman {
public:
  Kalman() {
    Q_angle = 0.02f;   // more trust in gyro dynamics
    Q_bias = 0.003f;   // bias update speed is okay
    R_measure = 0.01f; // trust accelerometer slightly more
    angle = 0.0f;
    bias = 0.0f;
    P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0.0f;
  }

  float getAngle(float newAngle, float newRate, float dt) {
    float rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;
    angle += K[0] * y;
    bias  += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
  }

  void setAngle(float newAngle) { angle = newAngle; }
  float getRate() { return bias; }

private:
  float Q_angle, Q_bias, R_measure;
  float angle, bias;
  float P[2][2];
};
Kalman kalmanX, kalmanY;
void mpu_setup(){
  mpu.initialize();
  mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO); // Use gyro X-axis as clock
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // or 500, 1000, 2000
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // or 4, 8, 16
  mpu.setDLPFMode(MPU6050_DLPF_BW_42); // 20 Hz low-pass filter
  mpu.setSleepEnabled(false); // Ensure MPU is awake
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  Serial.println("MPU6050 connected");

  // Calibrate initial angle using accel
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float accRoll = atan2(ay, az) * 180.0 / PI;
  float accPitch = atan2(ax, az) * 180.0 / PI;

  kalmanX.setAngle(accRoll);
  kalmanY.setAngle(accPitch);
}
void read_sensor(float dt){
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Accelerometer angles
  float accRoll  = atan2(ay - mpu_offsets[1], az - mpu_offsets[2]) * 180.0 / PI;
  float accPitch = atan2(ax - mpu_offsets[0], az - mpu_offsets[2]) * 180.0 / PI;

  // Gyroscope rates (deg/s)
  float gyroXrate = (gx - mpu_offsets[3]) / 131.0;
  float gyroYrate = (gy - mpu_offsets[4]) / 131.0;
  float gyroZrate = (gz - mpu_offsets[5]) / 131.0;  // yaw rate
  rollRate = gyroXrate;
  pitchRate = gyroYrate;
  // Kalman Filter
  if (mpu_calib_flag){
    roll  = kalmanX.getAngle(accRoll, gyroXrate, dt);
    pitch = kalmanY.getAngle(accPitch, gyroYrate, dt);
  }
  else{
    roll  = kalmanX.getAngle(accRoll, gyroXrate, dt) - mpu_offsets[6];
    pitch = kalmanY.getAngle(accPitch, gyroYrate, dt)- mpu_offsets[7];
    yawRate = gyroZrate;
  }
  // Serial.print("Roll: ");  Serial.print(roll);
  // Serial.print(" | Pitch: "); Serial.print(pitch);
  // Serial.print(" | Yaw rate: "); Serial.println(yawRate);
  // Output

}
void comput_angle_PID(float dt) {
  // Roll PID (angle control)
  roll_error_angle = rollInput - roll;
  rollIntegral_angle += 0.5 * (roll_error_angle + roll_pr_error_angle) * dt;
  float rollDerivative = (roll_error_angle - roll_pr_error_angle) / dt;
  roll_pid_rate = kp_roll_angle*roll_error_angle + ki_roll_angle*rollIntegral_angle +kd_roll_angle*rollDerivative;
  roll_pr_error_angle = roll_error_angle;

  // Pitch PID (angle control)
  pitch_error_angle = pitchInput - pitch;
  // Serial.print(pitchInput);Serial.print(" :: ");Serial.println(pitch_error);
  pitchIntegral_angle += 0.5 * (pitch_error_angle + pitch_pr_error_angle) * dt;
  float pitchDerivative = (pitch_error_angle - pitch_pr_error_angle) / dt;
  // Serial.println(kd_pitch*pitchDerivative);
  pitch_pid_rate = kp_pitch_angle*pitch_error_angle + ki_pitch_angle*pitchIntegral_angle +kd_pitch_angle*pitchDerivative;
  pitch_pr_error_angle = pitch_error_angle;

  // Yaw PID (rate control)
  // yaw_error = yawInput - yawRate;
  // yawIntegral += 0.5 * (yaw_error + yaw_pr_error) * dt;
  // float yawDerivative = (yaw_error - yaw_pr_error) / dt;
  // yaw_pid = kp_yaw*yaw_error + ki_yaw*yawIntegral +kd_yaw*yawDerivative;
  // yaw_pr_error = yaw_error;
}
void comput_rate_PID(float dt) {

  //------------------------------------
          // for final
  //----------------------------

  // // Roll PID (rate control)
  // roll_error_rate = roll_pid_rate - rollRate;
  // rollIntegral_rate += 0.5 * (roll_error_rate + roll_pr_error_rate) * dt;
  // float rollDerivative = (roll_error_rate - roll_pr_error_rate) / dt;
  // roll_pid_output = kp_roll_rate*roll_error_rate + ki_roll_rate*rollIntegral_rate +kd_roll_rate*rollDerivative;
  // roll_pr_error_rate = roll_error_rate;

  // // Pitch PID (rate control)
  // pitch_error_rate = pitch_pid_rate - pitchRate;
  // // Serial.print(pitchInput);Serial.print(" :: ");Serial.println(pitch_error);
  // pitchIntegral_rate += 0.5 * (pitch_error_rate + pitch_pr_error_rate) * dt;
  // float pitchDerivative = (pitch_error_rate - pitch_pr_error_rate) / dt;
  // // Serial.println(kd_pitch*pitchDerivative);
  // pitch_pid_output = kp_pitch_rate*pitch_error_rate + ki_pitch_rate*pitchIntegral_rate +kd_pitch_rate*pitchDerivative;
  // pitch_pr_error_rate = pitch_error_rate;

  // // Yaw PID (rate control)
  // // yaw_error = yawInput - yawRate;
  // // yawIntegral += 0.5 * (yaw_error + yaw_pr_error) * dt;
  // // float yawDerivative = (yaw_error - yaw_pr_error) / dt;
  // // yaw_pid = kp_yaw*yaw_error + ki_yaw*yawIntegral +kd_yaw*yawDerivative;
  // // yaw_pr_error = yaw_error;

  //-------------------------
  // tuning
  //----------------------

  // Roll rate PID (rate control)
  roll_error_rate = rollInput - rollRate;
  roll_error_rate =  (-1)*roll_error_rate
  rollIntegral_rate += 0.5 * (roll_error_rate + roll_pr_error_rate) * dt;
  float rollDerivative = (roll_error_rate - roll_pr_error_rate) / dt;
  roll_pid_output = kp_roll_rate*roll_error_rate + ki_roll_rate*rollIntegral_rate +kd_roll_rate*rollDerivative;
  roll_pr_error_rate = roll_error_rate;

  // Pitch PID (rate control)
  pitch_error_rate = pitchInput - pitchRate;
  pitch_error_rate = pitch_error_rate *(-1)
  // Serial.print(pitchInput);Serial.print(" :: ");Serial.println(pitch_error);
  pitchIntegral_rate += 0.5 * (pitch_error_rate + pitch_pr_error_rate) * dt;
  float pitchDerivative = (pitch_error_rate - pitch_pr_error_rate) / dt;
  // Serial.println(kd_pitch*pitchDerivative);
  pitch_pid_output = kp_pitch_rate*pitch_error_rate + ki_pitch_rate*pitchIntegral_rate +kd_pitch_rate*pitchDerivative;
  pitch_pr_error_rate = pitch_error_rate;

}
void processData(String data,char para) {
  float num = data.toFloat();
  num /= 1000;
  if (para == 'A'){
    kp_roll_rate = num;
  }
  else if (para == 'B'){
    ki_roll_rate = num; 
  }
  else if (para == 'C'){
    kd_roll_rate = num; 
  }
}
void bluetooth(){
  while (SerialBT.available()) {
    char receivedChar = SerialBT.read();

    if (receivedChar == 'A') { // end of message
      processData(inputString,receivedChar);
      inputString = ""; // clear buffer
    } 
    else if (receivedChar == 'B') { // end of message
      processData(inputString,receivedChar);
      inputString = ""; // clear buffer
    } 
    else if (receivedChar == 'C') { // end of message
      processData(inputString,receivedChar);
      inputString = ""; // clear buffer
    } 
    else {
      inputString += receivedChar;
    }
  }
}
void mpu_calib(){
  int16_t ax, ay, az, gx, gy, gz;
  for(int i = 0;i < nsamples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // mpu_offsets[0] += ax;
    // mpu_offsets[1] += ay;
    // mpu_offsets[2] += az;
    mpu_offsets[3] += gx;
    mpu_offsets[4] += gy;
    mpu_offsets[5] += gz;
  }

  for(int i = 0; i < 6; i++) {
    mpu_offsets[i] /= nsamples;
  }
  // mpu_offsets[2] += 9.81;

  for(int i = 0 ;i<nsamples ;i++){
    read_sensor(rate_loop_time);
    mpu_offsets[6] += roll;
    mpu_offsets[7] += pitch;
  }
  mpu_offsets[6] /= nsamples;
  mpu_offsets[7] /= nsamples;
  mpu_calib_flag = false;
  Serial.println("Calibration Done");
}


void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT");
  setupPPM();    // Initialize PPM input
  Wire.begin(21,22);
  delay(100);
  mpu_setup();
  mpu_calib();
  setupMotors();
  Serial.println("Setup Complete");
}
void loop() {
  CurrentTime = micros();
  read_sensor(rate_loop_time);
  bluetooth();
  if (CurrentTime - rate_lastLoopTime >= (rate_loop_time)*1000000){
    readPPM();
    comput_rate_PID(rate_loop_time);
    // if (CurrentTime - angle_lastLoopTime >= (angle_loop_time)*1000000){
    //   comput_angle_PID(angle_loop_time);
    //   angle_lastLoopTime = micros();
    // }
    mixMotorOutputs();
    writeMotors();
    rate_lastLoopTime = micros();
    if (count%200 == 0){
      Serial.print("pitch : ");Serial.print(pitch_error_rate);Serial.print(" | ");Serial.print(pitch_pid_output);
      Serial.print(" :: ");Serial.print("roll : ");Serial.print(roll_error_rate);Serial.print(" | ");Serial.print(roll_pid_output);
      Serial.print(" :: ");Serial.print("yaw : ");Serial.print(yaw_error_rate);Serial.print(" | ");Serial.print(yaw_pid_output);
      Serial.print(" :: ");Serial.print("m1out : ");Serial.print(m1Out);Serial.print(" :: ");Serial.print("m2Out : ");Serial.print(m2Out);Serial.print(" :: ");Serial.print("m3out : ");Serial.print(m3Out);Serial.print(" :: ");Serial.print("m4out : ");Serial.println(m4Out);
      // Serial.print("pitch : ");Serial.print(mpu_offsets[6]);Serial.print(" | ");Serial.print("roll : ");Serial.println(rollInput);
    }
    if (count%20 == 0){
      send = "kp :: " + String(kp_roll_rate) + " :: ki :: " + String(ki_roll_rate) + " :: kd :: " + String(kd_roll_rate);
      // send = "target roll rate" + String(rollInput) + " roll rate error " + String(rollRate);
      // Serial.println(send);
      // Serial.println(send);
      SerialBT.println(send);
    }
    count++;

  }

}





