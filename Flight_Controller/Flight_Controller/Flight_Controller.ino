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
float err[3] = {0, 0, 0}; // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float deltaErr[3] = {0, 0, 0}; // Error deltas in that order   : Yaw, Pitch, Roll
float errSum[3] = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float prevErr[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]

// PID coefficients
float kP[3] = {1.0, 1.2, 1.2};    // P coefficients in that order : Yaw, Pitch, Roll
float kI[3] = {0, 0, 0}; // I coefficients in that order : Yaw, Pitch, Roll
float kD[3] = {0, 30, 30};        // D coefficients in that order : Yaw, Pitch, Roll

const float pidLimit = 200; // Max effect the PID will have on motor speeds

// Define drone angles
float angs[3] = {0, 0, 0};

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
 * Function: cleanRxIn
 * 
 * This function creates a 16us deadband in the center to prevent pid errors
 * when the joystick values doesn't perfectly output 1500us (centered)
 * 
 * int rxIn: raw receiever input
 * 
 * returns: corrected pulse output (1000-2000)
 */
int correctRxIn(int rxIn) {
  return ((rxIn > 1492 && rxIn < 1508) ? 1500 : rxIn);
}

/*
 * Function: updatePIDconst
 * 
 * This function updates the P, I, or D constants based on what it receives from
 * the udp socket
 * 
 * char* udpData: message received from the udp socket
 * 
 * returns: true if pid was updated
 */
bool updatePIDconst(char* udpData) {
  // Select constant array
  float *selectedParam;
  switch (udpData[0]) {
    case 'p':
      selectedParam = kP; break;
    case 'i':
      selectedParam = kI; break;
    case 'd':
      selectedParam = kD; break;
    default:
      return false;
  }

  // Select yaw, pitch, or roll from that array
  int i = udpData[1] - 48;
  if (i < 0 || i > 2) {
    return false;
  }

  // Convert number in the string to a float
  char fstr[64];
  memcpy(fstr, udpData + 3, 64);
  float f = atof(fstr);

  // Update requested constant
  *(selectedParam + i) = f;

  return true;
}

/*
 * Function: calcPID
 * 
 * This function does the pid and motor speed calculations
 * 
 * returns null
 */
void calcPID() {
  float pid[3] = {0, 0, 0};
  int rawThrottle = constrain((int) rxPulseOut[2], 1000, 2000);
  int throttle = 0.5 * rawThrottle + 500;

  // Initialize motor commands with throttle
  for (int i = 0; i < 4; ++i) {
    mOut[i] = throttle;
  }

  // Do not calculate anything if throttle is 0
  if (rawThrottle >= 1012) {
    for (int i = 0; i < 3; ++i) {
      // PID = e.Kp + ∫e.Ki + Δe.Kd
      pid[i] = (err[i] * kP[i]) + (errSum[i] * kI[i]) + (deltaErr[i] * kD[i]);

      // Constrain values
      pid[i] = constrain(pid[i], -pidLimit, pidLimit);
    }

    // Calculate pulse duration for each ESC
    mOut[0] = throttle - pid[0] - pid[1] + pid[2];
    mOut[1] = throttle + pid[0] - pid[1] - pid[2];
    mOut[2] = throttle - pid[0] + pid[1] - pid[2];
    mOut[3] = throttle + pid[0] + pid[1] + pid[2];
  }

  // Prevent out-of-range-values
  for (int i = 0; i < 4; ++i) {
    mOut[i] = constrain(mOut[i], 1000, 2000);
  }
}

/*
 * Function: resetErr
 * 
 * This function resets the error values to 0
 * 
 * returns null
 */
void resetErr() {
  for (int i = 0; i < 3; ++i) {
    err[i] = 0;
    errSum[i] = 0;
    deltaErr[i] = 0;
    prevErr[i] = 0;
  }
}

/*
 * Function: calcErr
 * 
 * This function calculates the P, I, and D errors used for the PID controller
 * 
 * returns null
 */
void calcErr() {
  for (int i = 0; i < 3; ++i) {
    // Calculate current errors
    err[i] = angs[i] - pidSetPoints[i];

    // Only update error sum if integral coefficient is > 0
    if (kI[i] > 0) {
      // Calculate sum of errors: Integral coefficients
      errSum[i] += err[i];
  
      // Constrain values
      errSum[i] = constrain(errSum[i], -pidLimit / kI[i], pidLimit / kI[i]);
    }

    // Calculate error delta : Derivative coefficients
    deltaErr[i] = err[i] - prevErr[i];

    // Save current error as previous_error for next time
    prevErr[i] = err[i];
  }
}

/*
 * Function: getOrientation
 * 
 * This function calculates the drone orientation
 * 
 * returns null
 */
void getOrientation() {
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
  angs[0] = g.gyro.z * SENSORS_RADS_TO_DPS;

  float pitch = filter.getRoll(); // Pitch and roll are reversed because of sensor orientation
  angs[1] = (pitch < 0 ? 180 + pitch : pitch - 180);
  
  angs[2] = -filter.getPitch();
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
  Serial.print("\nConnected to network\nIP Address: ");
  Serial.println(WiFi.localIP());

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
  // Update orientation and pid when the filter timer is triggered
  if (xSemaphoreTake(filterTimerSem, 0) == pdTRUE) {
    // Get drone orientation
    getOrientation();
  
    // Get desired orientation
    pidSetPoints[0] = 0.12 * correctRxIn(rxPulseOut[3]) - 180;
    pidSetPoints[1] = -0.03 * correctRxIn(rxPulseOut[1]) + 45;
    pidSetPoints[2] = -0.03 * correctRxIn(rxPulseOut[0]) + 45;
  
    // Calculate errors
    calcErr();
  
    // Calculate PID and motor outputs
    calcPID();
  
    // Output motor speed
    for (int i = 0; i < 4; ++i) {
      MOTORS[i].writeMicroseconds(mOut[i]);
    }
  
    // Read UDP data and update PID
    char packetBuffer[128];
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int n = udp.read(packetBuffer, 128);
      packetBuffer[n] = 0;
      bool updated = updatePIDconst(packetBuffer);
      if (updated) {
        resetErr();
      }
    }
  }

  // Send quaternion data over udp when the udp timer is triggered
  if (xSemaphoreTake(udpTimerSem, 0) == pdTRUE) {
//    // Scale quaternion data from [-1, 1] to [0, 255]
//    char cw = 127.5 * qnw + 127.5;
//    char cx = 127.5 * qnx + 127.5;
//    char cy = 127.5 * qny + 127.5;
//    char cz = 127.5 * qnz + 127.5;
//
//    // Scale motor data from [0, 1000] to [0, 255]
//    char cm0 = mOut[0] * 0.255;
//    char cm1 = mOut[1] * 0.255;
//    char cm2 = mOut[2] * 0.255;
//    char cm3 = mOut[3] * 0.255;
//
//    // Scale control inputs from [1000, 2000] to [0, 255]
//    char ci0 = 0.255 * rxPulseOut[0] - 255;
//    char ci1 = 0.255 * rxPulseOut[1] - 255;
//    char ci2 = 0.255 * rxPulseOut[2] - 255;
//    char ci3 = 0.255 * rxPulseOut[3] - 255;
//
//    // Split timestamp into 4 bytes
//    unsigned long t = millis();
//    char ct0 = t & 255;
//    char ct1 = (t >> 8) & 255;
//    char ct2 = (t >> 16) & 255;

//    Serial.printf("yaw: %0.2f, pitch: %0.2f, roll: %0.2f\n", angs[0], angs[1], angs[2]);

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
