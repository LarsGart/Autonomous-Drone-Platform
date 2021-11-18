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
const char* udpAddr = "10.0.0.41"; // Jetson IP Address
const int udpPort = 44444;
bool sendQuat = false;

// Define timer variables
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Define motors
Servo MOTORS[4];

// Define notes and durations for Blackpink intro lol
#define BPM 130
#define QUARTER (1000.0 / (BPM / 60.0))
#define QUARTER_DOT (QUARTER + QUARTER / 2.0)
#define EIGHTH (QUARTER / 2.0)
#define SIXTNTH (QUARTER / 4.0)

const note_t notes[11] = {
  NOTE_E, NOTE_E, NOTE_E, NOTE_G, NOTE_Fs, NOTE_A,
  NOTE_B, NOTE_Bb, NOTE_B, NOTE_G, NOTE_E
};

const int dur[11] = {
  QUARTER, EIGHTH, EIGHTH, QUARTER_DOT, SIXTNTH, SIXTNTH,
  EIGHTH, EIGHTH, EIGHTH, EIGHTH, EIGHTH
};

// Define sensors
Adafruit_FXOS8700 fxos = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C fxas = Adafruit_FXAS21002C(0x0021002C);

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// Define sensor fusion filter
Adafruit_NXPSensorFusion filter;

// Define calibration data in EEPROM
Adafruit_Sensor_Calibration_EEPROM cal;

// Create UDP object
WiFiUDP udp;

/*
 * Function: onTimer
 * 
 * Interrupt handler for when the timer is triggered
 * 
 * returns: null
 */
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  sendQuat = true;
  portEXIT_CRITICAL_ISR(&timerMux);
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

void setup() {
  Serial.begin(115200);

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

  // Play startup sound
//  ledcSetup(15, 2000, 8);
//  ledcAttachPin(27, 15);
//  delay(500);
//  for (int i = 0; i < 11; ++i) {
//    ledcWriteNote(15, notes[i], 5);
//    ledcWrite(15, 255);
//    delay(100);
//    ledcWrite(15, 0);
//    delay(dur[i] - 100);
//  }
//  ledcDetachPin(27);
//  delay(1000);

  // Initialize motor outputs
  for (int i = 0; i < 4; ++i) {
    ESP32PWM::allocateTimer(i);
    MOTORS[i].setPeriodHertz(50);
    MOTORS[i].attach(MOTOR_GPIOS[i], 1000, 2000);
//    MOTORS[i].writeMicroseconds(1000);
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
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 55555, true);
  timerAlarmEnable(timer);

  // Connect to WiFi
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
  if (!fxos.begin() || !fxas.begin()) {
    Serial.println("Failed to find sensors");
    return;
  }
  accelerometer = fxos.getAccelerometerSensor();
  gyroscope = &fxas;
  magnetometer = fxos.getMagnetometerSensor();

  // Initialize sensor fusion
  filter.begin(100); 

  // Set I2C communication speed
  Wire.setClock(400000);

  
}

void loop() {
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

  // Get drone orientation
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);

  // Rotate quaternion to get true drone orientation
  float qnw = SQRTHALF * (qy - qx);
  float qnx = SQRTHALF * (qw + qz);
  float qny = SQRTHALF * (qz - qw);
  float qnz = SQRTHALF * (-qx - qy);

//  Serial.print("Quaternion: ");
//  Serial.print(qnw, 4);
//  Serial.print(", ");
//  Serial.print(qnx, 4);
//  Serial.print(", ");
//  Serial.print(qny, 4);
//  Serial.print(", ");
//  Serial.println(qnz, 4);
//  Serial.printf(
//    "qr=%0.4f+%0.4fi+%0.4fj+%0.4fk, qn=%0.4f+%0.4fi+%0.4fj+%0.4fk\n",
//    qw, qx, qy, qz, qnw, qnx, qny, qnz
//  );

  // Send quaternion data over udp at a rate of 20Hz
  if (sendQuat) {
    sendQuat = false;
    
    udp.beginPacket(udpAddr, udpPort);
    udp.printf("w%0.3fwa%0.3fab%0.3fbc%0.3fc\n", qnw, qnx, qny, qnz);
    udp.endPacket();
    Serial.printf("w%0.3fwa%0.3fab%0.3fbc%0.3fc\n", qnw, qnx, qny, qnz);
  }
  
  int t = constrain((int) rxPulseOut[2] - 1000, 0, 1000);
  int r = constrain((int) rxPulseOut[0] - 1500, -500, 500);
  int p = constrain((int) rxPulseOut[1] - 1500, -500, 500);
  int y = constrain((int) rxPulseOut[3] - 1500, -500, 500);

  int mOut[4] = {
    constrain(0.4 * t + 0.1 * r - 0.1 * p + 0.1 * y, 0, 1000),
    constrain(0.4 * t - 0.1 * r - 0.1 * p - 0.1 * y, 0, 1000),
    constrain(0.4 * t - 0.1 * r + 0.1 * p + 0.1 * y, 0, 1000),
    constrain(0.4 * t + 0.1 * r + 0.1 * p - 0.1 * y, 0, 1000)
  };

  for (int i = 0; i < 4; ++i) {
    MOTORS[i].writeMicroseconds(mOut[i] + 1000);
  }
}
