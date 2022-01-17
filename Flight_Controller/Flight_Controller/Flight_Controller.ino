/*
 * CHANNEL MAPPINGS
 * 
 * CH1: ROLL (LEFT -> 1000, RIGHT -> 2000)
 * CH2: PITCH (DOWN -> 1000, UP -> 2000)
 * CH3: THROTTLE (DOWN -> 1000, UP -> 2000)
 * CH4: YAW (LEFT -> 1000, RIGHT -> 2000)
 * 
 * MOTOR MAPPINGS
 * 
 *    0  ^  1
 *     \_|_/
 *     |   |
 *     |___|
 *     /   \
 *    3     2
 */

#include "driver/rmt.h"
#include <ESP32Servo.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// Math constants
#define SQRTHALF 0.70710678118

// Define GPIO pins for rx channels and motor outputs
const uint8_t RX_GPIOS[4] = {4, 16, 17, 5};
const uint8_t MOTOR_GPIOS[4] = {23, 19, 13, 18};

// Initialize data array for rx pulse inputs
volatile uint16_t rxPulseOut[4] = {1500, 1500, 1000, 1500};

// Define UDP parameters
const char* udpAddr = "10.0.0.36"; // Jetson IP Address
const int udpPort = 44444; // UDP port
const int udpSendRate = 20; // Rate at which it transmits in Hz

// Define timer variables
hw_timer_t *udpTimer = NULL;
hw_timer_t *filterTimer = NULL;
volatile SemaphoreHandle_t udpTimerSem, filterTimerSem;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Define motors
Servo MOTORS[4];
int mOut[4];

// Define sensors
Adafruit_FXOS8700 fxos = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C fxas = Adafruit_FXAS21002C(0x0021002C);

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// Define sensor fusion filter
const int filterUpdateRate = 200;
Adafruit_NXPSensorFusion filter;

// Define calibration data in EEPROM
Adafruit_Sensor_Calibration_EEPROM cal;

// Define calibration data
float aOffset[3];
float gOffset[3];
float mOffset[3];

// Create UDP object
WiFiUDP udp;

// PID set points
float pidSetPoints[3] = {0, 0, 0}; // Yaw, Pitch, Roll

// Errors
float err[3]; // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float deltaErr[3] = {0, 0, 0}; // Error deltas in that order   : Yaw, Pitch, Roll
float errSum[3] = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float prevErr[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]

// PID coefficients
float kP[3] = {4.0, 1.3, 1.3};    // P coefficients in that order : Yaw, Pitch, Roll
float kI[3] = {0.02, 0.04, 0.04}; // I coefficients in that order : Yaw, Pitch, Roll
float kD[3] = {0, 18, 18};        // D coefficients in that order : Yaw, Pitch, Roll

// Define drone quaternion
float qnw = 0, qnx = 0, qny = 0, qnz = 0;

// Define drone angles
float yaw = 0;
float pitch = 0;
float roll = 0;

/*
 * Function: onUDPTimer
 * 
 * Interrupt handler for when the udp timer is triggered
 * 
 * returns: null
 */
void IRAM_ATTR onUDPTimer() {
  xSemaphoreGiveFromISR(udpTimerSem, NULL);
}

/*
 * Function: onFilterTimer
 * 
 * Interrupt handler for when the filter timer is triggered
 * 
 * returns: null
 */
 void IRAM_ATTR onFilterTimer() {
  xSemaphoreGiveFromISR(filterTimerSem, NULL);
}

/*
 * Function: rmt_isr_handler
 * 
 * Interrupt handler for when a pulse is outputted from the rx
 * 
 * returns: null
 */
static void rmt_isr_handler(void* arg){
  uint32_t intr_st = RMT.int_st.val;
  
  for(uint8_t ch = 0; ch < 4; ++ch) {
    uint32_t ch_mask = BIT(ch * 3 + 1);

    if (!(intr_st & ch_mask)) continue;

    RMT.conf_ch[ch].conf1.rx_en = 0;
    RMT.conf_ch[ch].conf1.mem_owner = RMT_MEM_OWNER_TX;
    volatile rmt_item32_t* item = RMTMEM.chan[ch].data32;
    if (item) {
      rxPulseOut[ch] = item -> duration0;
    }

    RMT.conf_ch[ch].conf1.mem_wr_rst = 1;
    RMT.conf_ch[ch].conf1.mem_owner = RMT_MEM_OWNER_RX;
    RMT.conf_ch[ch].conf1.rx_en = 1;

    //clear RMT interrupt status.
    RMT.int_clr.val = ch_mask;
  }
}

/*
 * Function: sensorsInit
 * 
 * initializes the IMU
 * 
 * returns null
 */
void sensorsInit() {
  if (!fxos.begin() || !fxas.begin()) {
    Serial.println("Failed to find sensors");
    return;
  }
  accelerometer = fxos.getAccelerometerSensor();
  gyroscope = &fxas;
  magnetometer = fxos.getMagnetometerSensor();
}

/*
 * Function: calibrateAccelGyro
 * 
 * finds accelerometer and gyroscope biases and updates the offset arrays
 * 
 * returns null
 */
void calibrateAccelGyro() {
  for (int i = 0; i < 100; ++i) {
    sensors_event_t a, g;
    accelerometer -> getEvent(&a);
    gyroscope -> getEvent(&g);
  
    aOffset[0] += a.acceleration.x;
    aOffset[1] += a.acceleration.y;
    aOffset[2] += a.acceleration.z;
    gOffset[0] += g.gyro.x;
    gOffset[1] += g.gyro.y;
    gOffset[2] += g.gyro.z;
  }
  
  for (int i = 0; i < 3; ++i) {
    aOffset[i] /= 100.0;
    gOffset[i] /= 100.0;
  }
}

/*
 * Function: calibrateMagnetometer
 * 
 * communicates with the Jetson to get calibration data
 * or calibrate the magnetometer
 * 
 * returns null
 */
void calibrateMagnetometer() {
  udp.beginPacket(udpAddr, udpPort);
  udp.printf("calibration query");
  udp.endPacket();

  // Yield code until it receives hard iron offset or an 'n'
  char packetBuffer[128];
  bool calibrationNeeded = false;
  while (1) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int n = udp.read(packetBuffer, 128);
      packetBuffer[n] = 0;
      
      if (packetBuffer[0] == 'n') {
        calibrationNeeded = true;
      }
      break;
    }
  }

  if (calibrationNeeded) {
    // Send magnetometer data
    for (int i = 0; i < 200; ++i) {
      sensors_event_t m;
      magnetometer -> getEvent(&m);
      
      udp.beginPacket(udpAddr, udpPort);
      udp.printf(
        "%0.1f,%0.1f,%0.1f",
        m.magnetic.x,
        m.magnetic.y,
        m.magnetic.z
      );
      udp.endPacket();

      delay(100);
    }

    // Send "end" string to indicate end of data
    udp.beginPacket(udpAddr, udpPort);
    udp.printf("end");
    udp.endPacket();

    // Yield code until hard iron offset is received
    while (1) {
      int packetSize = udp.parsePacket();
      if (packetSize) {
        int n = udp.read(packetBuffer, 128);
        packetBuffer[n] = 0;
        break;
      }
    }
  }

  // Convert string to 3 floats
  mOffset[0] = atof(strtok(packetBuffer, ","));
  mOffset[1] = atof(strtok(NULL, ","));
  mOffset[2] = atof(strtok(NULL, ","));

  // echo it
  udp.beginPacket(udpAddr, udpPort);
  udp.printf(
    "Hard iron offset: %0.2f, %0.2f, %0.2f",
    mOffset[0],
    mOffset[1],
    mOffset[2]
  );
  udp.endPacket();
}

/*
 * Function: pidController
 * 
 * This function does the pid and motor speed calculations
 * 
 * returns null
 */
void calcPID() {
  float yawPID = 0;
  float pitchPID = 0;
  float rollPID = 0;
  int throttle = constrain((int) rxPulseOut[2], 1000, 2000);

  // Initialize motor commands with throttle
  mOut[0] = throttle;
  mOut[1] = throttle;
  mOut[2] = throttle;
  mOut[3] = throttle;

  // Do not calculate anything if throttle is 0
  if (throttle >= 1000) {
    // PID = e.Kp + ∫e.Ki + Δe.Kd
    yawPID = (err[0] * kP[0]) + (errSum[0] * kI[0]) + (deltaErr[0] * kD[0]);
    pitchPID = (err[1] * kP[1]) + (errSum[1] * kI[1]) + (deltaErr[1] * kD[1]);
    rollPID = (err[2] * kP[2]) + (errSum[2] * kI[2]) + (deltaErr[2] * kD[2]);

    // Keep values within acceptable range. TODO export hard-coded values in variables/const
    yawPID = constrain(yawPID, -400, 400);
    pitchPID = constrain(pitchPID, -400, 400);
    rollPID = constrain(rollPID, -400, 400);

    // Calculate pulse duration for each ESC
    mOut[0] = throttle - rollPID - pitchPID + yawPID;
    mOut[1] = throttle + rollPID - pitchPID - yawPID;
    mOut[2] = throttle - rollPID + pitchPID - yawPID;
    mOut[3] = throttle + rollPID + pitchPID + yawPID;
  }

  // Prevent out-of-range-values
  for (int i = 0; i < 4; ++i) {
    mOut[i] = constrain(mOut[i], 1000, 2000);
  }
}

//void calcErr() {
//    // Calculate current errors
//    err[0] = yaw - pid_set_points[YAW];
//    err[1] = pitch - pid_set_points[PITCH];
//    err[2] = roll - pid_set_points[ROLL];
//
//    // Calculate sum of errors : Integral coefficients
//    error_sum[YAW]   += errors[YAW];
//    error_sum[PITCH] += errors[PITCH];
//    error_sum[ROLL]  += errors[ROLL];
//
//    // Keep values in acceptable range
//    error_sum[YAW]   = minMax(error_sum[YAW],   -400/Ki[YAW],   400/Ki[YAW]);
//    error_sum[PITCH] = minMax(error_sum[PITCH], -400/Ki[PITCH], 400/Ki[PITCH]);
//    error_sum[ROLL]  = minMax(error_sum[ROLL],  -400/Ki[ROLL],  400/Ki[ROLL]);
//
//    // Calculate error delta : Derivative coefficients
//    delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
//    delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
//    delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];
//
//    // Save current error as previous_error for next time
//    previous_error[YAW]   = errors[YAW];
//    previous_error[PITCH] = errors[PITCH];
//    previous_error[ROLL]  = errors[ROLL];
//}

/*
 * Function: getOrientation
 * 
 * This function calculates the drone orientation
 * 
 * returns null
 */
void getOrientation() {
  // Update the filter when the filter timer is triggered
  if (xSemaphoreTake(filterTimerSem, 0) == pdTRUE) {
    // Read sensor values
    sensors_event_t a, g, m;
    accelerometer -> getEvent(&a);
    gyroscope -> getEvent(&g);
    magnetometer -> getEvent(&m);
  
    // Calibrate sensor values
    cal.calibrate(a);
    cal.calibrate(g);
    cal.calibrate(m);
  
    // Update sensor fusion filter
    filter.update(
      g.gyro.x * SENSORS_RADS_TO_DPS, g.gyro.y * SENSORS_RADS_TO_DPS, g.gyro.z * SENSORS_RADS_TO_DPS,
      a.acceleration.x, a.acceleration.y, a.acceleration.z,
      m.magnetic.x, m.magnetic.y, m.magnetic.z
    );

    // Get drone angles (angular velocity for yaw)
    yaw = g.gyro.z * SENSORS_RADS_TO_DPS;
    
    roll = filter.getPitch(); // pitch and roll are reversed because of sensor orientation

    float rawPitch = filter.getRoll();
    pitch = (rawPitch < 0 ? -180 - rawPitch : 180 - rawPitch);
  }
}

void setup() {
  Serial.begin(115200);

  // Indicate start of setup by turning on onboard LED
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  // Initialize RMT to read receiver outputs
  for (uint8_t ch = 0; ch < 4; ++ch) {
    rmt_config_t rmt_cfg;
    
    rmt_cfg.channel = (rmt_channel_t) ch;
    rmt_cfg.gpio_num = (gpio_num_t) RX_GPIOS[ch];
    rmt_cfg.clk_div = 80;
    rmt_cfg.mem_block_num = 1;
    rmt_cfg.rmt_mode = RMT_MODE_RX;
    rmt_cfg.rx_config.filter_en = true;
    rmt_cfg.rx_config.filter_ticks_thresh = 100;
    rmt_cfg.rx_config.idle_threshold = 3500;

    rmt_config(&rmt_cfg);
    rmt_set_rx_intr_en((rmt_channel_t) ch, true);
    rmt_rx_start((rmt_channel_t) ch, 1);
  }

  rmt_isr_register(rmt_isr_handler, NULL, 0, NULL);

  // Initialize motor outputs
  for (int i = 0; i < 4; ++i) {
    ESP32PWM::allocateTimer(i);
    MOTORS[i].setPeriodHertz(50);
    MOTORS[i].attach(MOTOR_GPIOS[i], 1000, 2000);
    MOTORS[i].writeMicroseconds(1000);
  }

  // Enter calibration mode if throttle is maxed
  delay(500);
  if (rxPulseOut[2] > 1900) {
    while (1) {
      for (int i = 0; i < 4; ++i) {
        MOTORS[i].writeMicroseconds(rxPulseOut[2]);
      }
    }
  }

  // Initialize timer for UDP transmission
  udpTimerSem = xSemaphoreCreateBinary();
  udpTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(udpTimer, &onUDPTimer, true);
  timerAlarmWrite(udpTimer, (int) (1000000.0 / udpSendRate), true);
  timerAlarmEnable(udpTimer);

  // Initialize timer for filter updating
  filterTimerSem = xSemaphoreCreateBinary();
  filterTimer = timerBegin(1, 80, true);
  timerAttachInterrupt(filterTimer, &onFilterTimer, true);
  timerAlarmWrite(filterTimer, (int) (1000000.0 / filterUpdateRate), true);
  timerAlarmEnable(filterTimer);

  // Connect to WiFi
  WiFi.setHostname("BLACKPINK DRONE");
  WiFi.begin("FBI Van", "Can'tGue$$Thi$");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected to network\nIP Address: %s\n", WiFi.localIP());

  // Create UDP
  udp.begin(udpPort);

  // Load sensor calibration data if it exists
  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (!cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }

  // Initialize sensors
  sensorsInit();

  // Calibrate accelerometer and gyroscope
  calibrateAccelGyro();

  // Calibrate magnetometer
  //calibrateMagnetometer();

  // Initialize sensor fusion
  filter.begin(filterUpdateRate); 

  // Set I2C communication speed
  Wire.setClock(400000);

  // Indicate setup is complete by turning off onboard LED
  digitalWrite(2, LOW);
}

void loop() {
  // Get drone orientation
  getOrientation();
  
//  int t = constrain((int) rxPulseOut[2] - 1000, 0, 1000);
//  int r = constrain((int) rxPulseOut[0] - 1500, -500, 500);
//  int p = constrain((int) rxPulseOut[1] - 1500, -500, 500);
//  int y = constrain((int) rxPulseOut[3] - 1500, -500, 500);
//
//  int mOut[4] = {
//    constrain(0.4 * t + 0.1 * r - 0.1 * p + 0.1 * y, 0, 1000),
//    constrain(0.4 * t - 0.1 * r - 0.1 * p - 0.1 * y, 0, 1000),
//    constrain(0.4 * t - 0.1 * r + 0.1 * p + 0.1 * y, 0, 1000),
//    constrain(0.4 * t + 0.1 * r + 0.1 * p - 0.1 * y, 0, 1000)
//  };
//
//  for (int i = 0; i < 4; ++i) {
//    MOTORS[i].writeMicroseconds(mOut[i] + 1000);
//  }

  // Send quaternion data over udp when the udp timer is triggered
  if (xSemaphoreTake(udpTimerSem, 0) == pdTRUE) {
    // Scale quaternion data from [-1, 1] to [0, 255]
    char cw = 127.5 * qnw + 127.5;
    char cx = 127.5 * qnx + 127.5;
    char cy = 127.5 * qny + 127.5;
    char cz = 127.5 * qnz + 127.5;

    // Scale motor data from [0, 1000] to [0, 255]
    char cm0 = mOut[0] * 0.255;
    char cm1 = mOut[1] * 0.255;
    char cm2 = mOut[2] * 0.255;
    char cm3 = mOut[3] * 0.255;

    // Scale control inputs from [1000, 2000] to [0, 255]
    char ci0 = 0.255 * rxPulseOut[0] - 255;
    char ci1 = 0.255 * rxPulseOut[1] - 255;
    char ci2 = 0.255 * rxPulseOut[2] - 255;
    char ci3 = 0.255 * rxPulseOut[3] - 255;

    // Split timestamp into 4 bytes
    unsigned long t = millis();
    char ct0 = t & 255;
    char ct1 = (t >> 8) & 255;
    char ct2 = (t >> 16) & 255;

    Serial.printf("yaw: %0.2f, pitch: %0.2f, roll: %0.2f\n", yaw, pitch, roll);

//    udp.beginPacket(udpAddr, udpPort);
//    udp.printf(
//      "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
//      cw, cx, cy, cz,
//      cm0, cm1, cm2, cm3,
//      ci0, ci1, ci2, ci3,
//      ct0, ct1, ct2
//    );
//    udp.endPacket();

//    udp.beginPacket(udpAddr, udpPort);
//    udp.printf(
//      "%0.3f,%0.3f,%0.3f,%0.3f",
//      qnw, qnx, qny, qnz
//    );
//    udp.endPacket();

//    udp.beginPacket(udpAddr, udpPort);
//    udp.printf(
//      "gx: %0.2f, gy: %0.2f, gz: %0.2f\nax: %0.2f, ay: %0.2f, az: %0.2f\nmx: %0.2f, my: %0.2f, mz: %0.2f\nqw: %0.2f, qx: %0.2f, qy: %0.2f, qz: %0.2f\n", 
//      g.gyro.x * SENSORS_RADS_TO_DPS,
//      g.gyro.y * SENSORS_RADS_TO_DPS,
//      g.gyro.z * SENSORS_RADS_TO_DPS,
//      a.acceleration.x,
//      a.acceleration.y,
//      a.acceleration.z,
//      m.magnetic.x,
//      m.magnetic.y,
//      m.magnetic.z,
//      qnw, qnx, qny, qnz
//    );
//    udp.endPacket();

//    udp.beginPacket(udpAddr, udpPort);
//    udp.printf(
//      "%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f",
//      g.gyro.x, g.gyro.y, g.gyro.z,
//      a.acceleration.x, a.acceleration.y, a.acceleration.z,
//      m.magnetic.x, m.magnetic.y, m.magnetic.z
//    );
//    udp.endPacket();
  }
}
