#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
// #include "BluetoothSerial.h"

//motor_output_variables
#define MOTOR1_PIN 26 // Top Right motor (motor 1)
#define MOTOR2_PIN 25  // Top Left motor (motor 2)
#define MOTOR3_PIN 33  // Bottom Left motor (motor 3)
#define MOTOR4_PIN 32  // Bottom Right motor (motor 4)
#define MOTOR1_CHANNEL 0
#define MOTOR2_CHANNEL 1
#define MOTOR3_CHANNEL 2
#define MOTOR4_CHANNEL 3
#define MOTOR_PWM_FREQ 400
#define MOTOR_PWM_RESOLUTION 12 // 12-bit resolution for duty cycle (0~4096)
float m1Out = 0, m2Out = 0, m3Out = 0, m4Out = 0;

//ppm_variables
#define PPM_PIN 5
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
bool armed = false;
float roll;
float pitch;
float rollRate;
float pitchRate;
float yawRate;
float actual_rate_loop_time = 0;
float actual_angle_loop_time = 0;

//time_variable
float angle_loop_time = 0.0025;
float rate_loop_time = 0.002;
unsigned long rate_lastLoopTime = 0;
unsigned long angle_lastLoopTime = 0;
int reset_angle_loop_time = 0;
int reset_rate_loop_time = 0;
unsigned long CurrentTime = 0;
unsigned long loop_start_time = 0;
unsigned long loop_end_time = 0;

//pid variable
float roll_rate_error = 0; float pr_roll_rate_error = 0;
float pitch_rate_error = 0; float pr_pitch_rate_error = 0;
float yaw_rate_error = 0; float pr_yaw_rate_error = 0;
float kp_roll_rate = 2.5 ; float kp_pitch_rate = 0 ; float kp_yaw_rate = 2.5;
float ki_roll_rate = 0.185 ; float ki_pitch_rate = 0 ; float ki_yaw_rate = 0.19;
float kd_roll_rate = -0.006 ; float kd_pitch_rate = 0 ; float kd_yaw_rate = -0.009;
float pitch_pid_output ; float roll_pid_output ; float yaw_pid_output ;
float rollIntegral_rate = 0;float pitchIntegral_rate = 0;float yawIntegral_rate =0;

float roll_angle_error = 0; float pr_roll_angle_error = 0;
float pitch_angle_error = 0; float pr_pitch_angle_error = 0;
float yaw_angle_error = 0; float pr_yaw_angle_error = 0;
float kp_roll_angle = 0.0 ; float kp_pitch_angle = 0.0 ; float kp_yaw_angle = 0;
float ki_roll_angle = 0.0 ; float ki_pitch_angle = 0.00 ; float ki_yaw_angle = 0;
float kd_roll_angle = 0.0 ; float kd_pitch_angle = 0.00 ; float kd_yaw_angle = 0;
float pitch_pid_rate = 0; float roll_pid_rate = 0; float yaw_pid_rate = 0;
float rollIntegral_angle = 0;float pitchIntegral_angle = 0;float yawIntegral_angle =0;
float MAX_RATE = 150; 

//pid tunning variable
String inputString = "";
String send = "";

//mpu calibration variables
float mpu_offsets[8] = {0.00,0.00,0.00,237.46,-115.78,-170.18,-4.11,1.21};
int nsamples = 5000;
bool mpu_calib_flag = false;

//debugging variables
int count = 0;

//communication variable
int sda = 22;
int scl = 21;

// ---------- WiFi ----------
const char* ssid = "abc";
const char* password = "12344321";
String htmlPage;

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

MPU6050 mpu;
// BluetoothSerial SerialBT;


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
  rollInput = map(rollInput,1000,2000,-60,60);
  pitchInput = map(pitchInput,1000,2000,-60,60);
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
  // mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  mpu.getMotion6(&ay, &ax, &az, &gy, &gx, &gz);
  az = (-1)*az;
  gz = (-1)*gz;
  float accRoll = atan2(ay, az) * 180.0 / PI;
  float accPitch = atan2(ax, az) * 180.0 / PI;

  kalmanX.setAngle(accRoll);
  kalmanY.setAngle(accPitch);
}
void read_sensor(float dt){
  int16_t ax, ay, az, gx, gy, gz;
  // mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  mpu.getMotion6(&ay, &ax, &az, &gy, &gx, &gz);
  az = (-1)*az;
  gz = (-1)*gz;
  

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
  roll_angle_error = rollInput - roll;
  rollIntegral_angle += 0.5 * (roll_angle_error + pr_roll_angle_error) * dt;
  rollIntegral_angle = constrain(rollIntegral_angle, -50, 50);
  if (ki_roll_angle == 0){
    rollIntegral_angle = 0;
  }
  float rollDerivative = (pr_roll_angle_error - roll_angle_error) / dt;
  roll_pid_rate = kp_roll_angle*roll_angle_error + ki_roll_angle*rollIntegral_angle +kd_roll_angle*rollDerivative;
  pr_roll_angle_error = roll_angle_error;
  roll_pid_rate = constrain(roll_pid_rate, -MAX_RATE, MAX_RATE);

  // Pitch PID (angle control)
  pitch_angle_error = pitchInput - pitch;
  pitchIntegral_angle += 0.5 * (pitch_angle_error + pr_pitch_angle_error) * dt;
  pitchIntegral_angle = constrain(pitchIntegral_angle, -50, 50);
  if (ki_pitch_angle == 0){
    pitchIntegral_angle = 0;
  }
  float pitchDerivative = (pr_pitch_angle_error - pitch_angle_error) / dt;
  pitch_pid_rate = kp_pitch_angle*pitch_angle_error + ki_pitch_angle*pitchIntegral_angle +kd_pitch_angle*pitchDerivative;
  pr_pitch_angle_error = pitch_angle_error;
  pitch_pid_rate = constrain(pitch_pid_rate, -MAX_RATE, MAX_RATE);
}
void comput_rate_PID(float dt) {

  // Roll PID (rate control)
  // roll_rate_error = roll_pid_rate - rollRate;
  roll_rate_error = rollInput - rollRate;
  rollIntegral_rate += 0.5 * (roll_rate_error + pr_roll_rate_error) * dt;
  rollIntegral_rate = constrain(rollIntegral_rate, -50, 50);
  if (ki_roll_rate == 0){
    rollIntegral_rate = 0;
  }
  float rollDerivative = (pr_roll_rate_error - roll_rate_error) / dt;
  roll_pid_output = kp_roll_rate*roll_rate_error + ki_roll_rate*rollIntegral_rate +kd_roll_rate*rollDerivative;
  pr_roll_rate_error = roll_rate_error;

  // Pitch PID (rate control)
  // pitch_rate_error = pitch_pid_rate - pitchRate;
  pitch_rate_error = (-1)*(pitchInput - pitchRate);
  pitchIntegral_rate += 0.5 * (pitch_rate_error + pr_pitch_rate_error) * dt;
  pitchIntegral_rate = constrain(pitchIntegral_rate, -50, 50);
  if (ki_pitch_rate == 0){
    pitchIntegral_rate = 0;
  }
  float pitchDerivative = (pr_pitch_rate_error - pitch_rate_error) / dt;
  pitch_pid_output = kp_pitch_rate*pitch_rate_error + ki_pitch_rate*pitchIntegral_rate +kd_pitch_rate*pitchDerivative;
  pr_pitch_rate_error = pitch_rate_error;

}
void mpu_calib(){
  int16_t ax, ay, az, gx, gy, gz;
  for(int i = 0;i < nsamples; i++) {
    // mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mpu.getMotion6(&ay, &ax, &az, &gy, &gx, &gz);
    az = (-1)*az;
    gz = (-1)*gz;

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
void setup_html_page(){
    htmlPage = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
    <title>Drone Telemetry Dashboard</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        body { font-family: Arial; text-align:center; margin:20px; background:#f2f2f2; }
        .container { display: grid; grid-template-rows: 1fr auto 1fr; gap: 15px; max-width: 1400px; margin: 0 auto; }
        .graph-row { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }
        .graph-section { background: white; padding: 10px; border-radius: 10px; box-shadow: 0 0 5px gray; }
        .graph-title { font-size: 12px; margin-top: 5px; text-align: center; line-height: 1.2; }
        canvas { width: 100%; height: 250px; }
        .controls { background: white; padding: 15px; border-radius: 10px; box-shadow: 0 0 5px gray; display: grid; grid-template-rows: repeat(5, auto); gap: 10px; }
        .pid-row { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; align-items: end; }
        .pid-group-left { text-align: left; }
        .pid-group-right { text-align: right; }
        .pid-group h5 { margin: 0 0 5px 0; }
        input { width: 80px; margin: 2px; padding: 3px; }
        .send-btn { justify-self: center; padding: 5px 10px; font-size: 16px; }
        .freq-row { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; justify-items: center; }
        .freq-group { text-align: left; }
        .motor-row { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; justify-items: center; }
        .motor-group { text-align: left; }
        .motor-group label { display: block; margin-bottom: 5px; }
        span { background: #e0e0e0; padding: 3px 6px; border-radius: 4px; }
    </style>
    </head>
    <body>
    <h2>Drone Telemetry Dashboard</h2>

    <div class="container">
    <div class="graph-row">
        <div class="graph-section">
        <canvas id="rollRate"></canvas>
        <div class="graph-title">
            desired roll rate <span id="curr_roll_rate_des">0</span> actual roll rate <span id="curr_roll_rate_act">0</span>
        </div>
        </div>
        <div class="graph-section">
        <canvas id="pitchRate"></canvas>
        <div class="graph-title">
            desired pitch rate <span id="curr_pitch_rate_des">0</span> actual pitch rate <span id="curr_pitch_rate_act">0</span>
        </div>
        </div>
    </div>

    <div class="controls">
        <div class="pid-row">
        <div class="pid-group pid-group-left">
            <h5>Roll Rate</h5>
            Kp <input id="kp_roll_rate" value="0"><br>
            Ki <input id="ki_roll_rate" value="0"><br>
            Kd <input id="kd_roll_rate" value="0">
        </div>
        <div class="pid-group pid-group-right">
            <h5>Pitch Rate</h5>
            Kp <input id="kp_pitch_rate" value="0"><br>
            Ki <input id="ki_pitch_rate" value="0"><br>
            Kd <input id="kd_pitch_rate" value="0">
        </div>
        </div>

        <button class="send-btn" onclick="sendPID()">Send</button>

        <div class="pid-row">
        <div class="pid-group pid-group-left">
            <h5>Roll Angle</h5>
            Kp <input id="kp_roll_angle" value="0"><br>
            Ki <input id="ki_roll_angle" value="0"><br>
            Kd <input id="kd_roll_angle" value="0">
        </div>
        <div class="pid-group pid-group-right">
            <h5>Pitch Angle</h5>
            Kp <input id="kp_pitch_angle" value="0"><br>
            Ki <input id="ki_pitch_angle" value="0"><br>
            Kd <input id="kd_pitch_angle" value="0">
        </div>
        </div>

        <div class="freq-row">
        <div class="freq-group">
            <label>Real rate frequency <span id="actual_rate_loop_time">0</span></label>
        </div>
        <div class="freq-group">
            <label>Real angle frequency <span id="actual_angle_loop_time">0</span></label>
        </div>
        </div>

        <div class="motor-row">
        <div class="motor-group">
            <label>M1 thrust <span id="M1_pwm">0</span></label>
            <label>M2 thrust <span id="M2_pwm">0</span></label>
        </div>
        <div class="motor-group">
            <label>M3 thrust <span id="M3_pwm">0</span></label>
            <label>M4 thrust <span id="M4_pwm">0</span></label>
        </div>
        </div>
    </div>

    <div class="graph-row">
        <div class="graph-section">
        <canvas id="rollTheta"></canvas>
        <div class="graph-title">
            desired roll angle <span id="curr_roll_theta_des">0</span> actual roll angle <span id="curr_roll_theta_act">0</span>
        </div>
        </div>
        <div class="graph-section">
        <canvas id="pitchTheta"></canvas>
        <div class="graph-title">
            desired pitch angle <span id="curr_pitch_theta_des">0</span> actual pitch angle <span id="curr_pitch_theta_act">0</span>
        </div>
        </div>
    </div>
    </div>

    <script>
    let ws = new WebSocket("ws://" + location.hostname + ":81/");
    let charts = {};
    let maxPoints = 100;

    function makeChart(id, label1, label2, ylabel) {
    return new Chart(document.getElementById(id), {
        type: 'line',
        data: { labels: [], datasets: [
        {label: label1, data: [], borderColor: 'blue', fill: false},
        {label: label2, data: [], borderColor: 'red', fill: false}
        ]},
        options: {
        animation: false,
        elements: {
            point: {
            radius: 0
            }
        },
        scales: {x:{title:{display:true,text:'Time(s)'}}, y:{title:{display:true,text: ylabel}}}
        }
    });
    }

    window.onload = () => {
    charts.rollRate = makeChart("rollRate","desired roll rate","actual roll rate","(degree/sec)");
    charts.pitchRate = makeChart("pitchRate","desired pitch rate","actual pitch rate","(degree/sec)");
    charts.rollTheta = makeChart("rollTheta","desired roll angle","actual roll angle","(degree)");
    charts.pitchTheta = makeChart("pitchTheta","desired pitch angle","actual pitch angle","(degree)");
    };

    ws.onmessage = (e) => {
    let d = JSON.parse(e.data);
    let t = d.time.toFixed(2);

    function upd(chart, y1, y2, desId, actId) {
        chart.data.labels.push(t);
        chart.data.datasets[0].data.push(y1);
        chart.data.datasets[1].data.push(y2);
        if (chart.data.labels.length > maxPoints) {
        chart.data.labels.shift();
        chart.data.datasets[0].data.shift();
        chart.data.datasets[1].data.shift();
        }
        chart.update();
        // Update current values
        document.getElementById(desId).innerText = y1.toFixed(3);
        document.getElementById(actId).innerText = y2.toFixed(3);
    }

    upd(charts.rollRate, d.roll_pid_rate, d.rollRate, 'curr_roll_rate_des', 'curr_roll_rate_act');
    upd(charts.pitchRate, d.pitch_pid_rate, d.pitchRate, 'curr_pitch_rate_des', 'curr_pitch_rate_act');
    upd(charts.rollTheta, d.rollInput, d.roll, 'curr_roll_theta_des', 'curr_roll_theta_act');
    upd(charts.pitchTheta, d.pitchInput, d.pitch, 'curr_pitch_theta_des', 'curr_pitch_theta_act');

    document.getElementById('M1_pwm').innerText = d.m1Out;
    document.getElementById('M2_pwm').innerText = d.m2Out;
    document.getElementById('M3_pwm').innerText = d.m3Out;
    document.getElementById('M4_pwm').innerText = d.m4Out;
    document.getElementById('actual_rate_loop_time').innerText = d.actual_rate_loop_time.toFixed(3);
    document.getElementById('actual_angle_loop_time').innerText = d.actual_angle_loop_time.toFixed(3);
    };

    function sendPID() {
    let payload = {
        kp_roll_rate: parseFloat(document.getElementById('kp_roll_rate').value),
        ki_roll_rate: parseFloat(document.getElementById('ki_roll_rate').value),
        kd_roll_rate: parseFloat(document.getElementById('kd_roll_rate').value),
        kp_roll_angle: parseFloat(document.getElementById('kp_roll_angle').value),
        ki_roll_angle: parseFloat(document.getElementById('ki_roll_angle').value),
        kd_roll_angle: parseFloat(document.getElementById('kd_roll_angle').value),
        kp_pitch_rate: parseFloat(document.getElementById('kp_pitch_rate').value),
        ki_pitch_rate: parseFloat(document.getElementById('ki_pitch_rate').value),
        kd_pitch_rate: parseFloat(document.getElementById('kd_pitch_rate').value),
        kp_pitch_angle: parseFloat(document.getElementById('kp_pitch_angle').value),
        ki_pitch_angle: parseFloat(document.getElementById('ki_pitch_angle').value),
        kd_pitch_angle: parseFloat(document.getElementById('kd_pitch_angle').value)
    };
    ws.send(JSON.stringify(payload));
    }
    </script>
    </body></html>
    )rawliteral";
}
void handleRoot() {
    server.send(200, "text/html", htmlPage); 
}
void handleWebSocketMessage(uint8_t num, uint8_t *payload, size_t length) {
  String msg = String((char*)payload).substring(0, length);
  DynamicJsonDocument doc(512);
  DeserializationError err = deserializeJson(doc, msg);
  if (!err) {
    kp_roll_rate = doc["kp_roll_rate"];
    ki_roll_rate = doc["ki_roll_rate"];
    kd_roll_rate = doc["kd_roll_rate"];
    kp_roll_angle = doc["kp_roll_angle"];
    ki_roll_angle = doc["ki_roll_angle"];
    kd_roll_angle = doc["kd_roll_angle"];
    kp_pitch_rate = doc["kp_pitch_rate"];
    ki_pitch_rate = doc["ki_pitch_rate"];
    kd_pitch_rate = doc["kd_pitch_rate"];
    kp_pitch_angle = doc["kp_pitch_angle"];
    ki_pitch_angle = doc["ki_pitch_angle"];
    kd_pitch_angle = doc["kd_pitch_angle"];
    Serial.println("PID values updated from webpage!");
  }
}
void sendTelemetry() {
    DynamicJsonDocument doc(1024);
    doc["time"] = (micros())/1000000;
    // doc["roll_pid_rate"] = roll_pid_rate;
    doc["roll_pid_rate"] = rollInput;
    doc["rollRate"] = rollRate;
    // doc["pitch_pid_rate"] = pitch_pid_rate;
    doc["pitch_pid_rate"] = pitchInput;
    doc["pitchRate"] = pitchRate;
    doc["rollInput"] = rollInput;
    doc["roll"] = roll;
    doc["pitchInput"] = pitchInput;
    doc["pitch"] = pitch;
    doc["m1Out"] = (int)m1Out;
    doc["m2Out"] = (int)m2Out;
    doc["m3Out"] = (int)m3Out;
    doc["m4Out"] = (int)m4Out;
    doc["actual_rate_loop_time"] = actual_rate_loop_time;
    doc["actual_angle_loop_time"] = actual_angle_loop_time;

    String json;
    serializeJson(doc, json);
    webSocket.broadcastTXT(json);
}
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_TEXT) handleWebSocketMessage(num, payload, length);
}


void setup() {
  setupMotors();
  writeMotors(); //just for safety
  Serial.begin(115200);
  setupPPM();    // Initialize PPM input
  Wire.begin(sda,scl);
  delay(100);
  mpu_setup();
  if (mpu_calib_flag){
    for (int i =0; i<8;i++){
    mpu_offsets[i]=0;
    }
    mpu_calib(); //#i have hard coded offsets
    for (int i =0; i<8;i++){
      Serial.print(mpu_offsets[i]);
      Serial.print(",");
    }
    Serial.println("");
  }  
  Serial.println("Setup Complete");
  setup_html_page();
  WiFi.begin(ssid, password);
  Serial.print("Connecting to wifi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
    }
  Serial.println("\nConnected to WiFi!");
  Serial.println(WiFi.localIP());
  server.on("/", handleRoot);
  server.begin();
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}
void loop() {
  server.handleClient();
  webSocket.loop();
  CurrentTime = micros();
  if (CurrentTime - rate_lastLoopTime >= (rate_loop_time)*1000000){
    read_sensor(((CurrentTime - rate_lastLoopTime)*1.0)/1000000);
    reset_rate_loop_time++;
    actual_rate_loop_time += (CurrentTime - rate_lastLoopTime);
    readPPM();
    comput_rate_PID(rate_loop_time);
    // printf("%d\n",CurrentTime - angle_lastLoopTime);
    if (CurrentTime - angle_lastLoopTime >= (angle_loop_time)*1000000){
    // if (true){
      // Serial.println(CurrentTime - angle_lastLoopTime);

      reset_angle_loop_time++;
      actual_angle_loop_time += (CurrentTime - angle_lastLoopTime);
      comput_angle_PID(angle_loop_time);
      angle_lastLoopTime = micros();
    }
    mixMotorOutputs();
    writeMotors();
    if (count%50 == 0){
      actual_rate_loop_time = (1000000*reset_rate_loop_time)/actual_rate_loop_time;
      actual_angle_loop_time = (1000000*reset_angle_loop_time)/actual_angle_loop_time;
      sendTelemetry();
      actual_rate_loop_time = 0;
      actual_angle_loop_time = 0;
      reset_rate_loop_time = 0;
      reset_angle_loop_time = 0;
    }
    count++;
    rate_lastLoopTime = micros();
    // Serial.print(mpu_offsets[0]);Serial.print(",");Serial.print(mpu_offsets[1]);Serial.print(",");Serial.print(mpu_offsets[2]);Serial.print(",");Serial.print(mpu_offsets[3]);Serial.print(",");Serial.print(mpu_offsets[4]);Serial.print(",");Serial.print(mpu_offsets[5]);Serial.print(",");Serial.print(mpu_offsets[6]);Serial.print(",");Serial.println(mpu_offsets[7]);
  }

}
