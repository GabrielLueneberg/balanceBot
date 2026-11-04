#include <Arduino.h>
#include <Wire.h>

//Pinos
#define IMU_SDA 21
#define IMU_SCL 22
#define STEP_L  4
#define DIR_L   15
#define STEP_R  18
#define DIR_R   19

//Direção das rodas
#define INVERT_L   false
#define INVERT_R   true

//Frequência da ISR
#define STEP_TIMER_FREQ_HZ 50000

//Período das tasks
#define CONTROL_PERIOD_MS    5
#define TELEMETRY_PERIOD_MS  20
#define CMD_PAUSE_MS         2000

//Definições da IMU
#define IMU_DLPF_CFG   0x03
#define GYRO_SIGN     -1.0f

//Limites e filtros
#define MAX_SPEED        4000.0f
#define MAX_ACCEL        40000.0f
#define GYRO_LPF_ALPHA   0.40f

//Ângulos de operação
#define FALL_ANGLE     35.0f
#define ARM_ANGLE      5.0f

//Estruturas de dados
typedef struct {
  uint8_t step_pin;
  uint8_t dir_pin;
  bool invert;
  volatile int32_t target_speed;
  volatile uint32_t acc;
} StepperMotor;

typedef struct {
  float ax, az;
  float gy;
} ImuSample;

typedef struct {
  float angle;
  float rate;
  float speed;
  float loop_us;
  bool  armed;
} TelemetryData;

//Parâmetros de controle
float Kp = 750.0f;
float Kd = 100.0f;
float angleZero = -9.4f;
volatile bool armEnabled = false;

// ================= MOTORES =================
//Instâncias dos motores e timer
StepperMotor motorL = {STEP_L, DIR_L, INVERT_L, 0, 0};
StepperMotor motorR = {STEP_R, DIR_R, INVERT_R, 0, 0};
hw_timer_t *stepTimer = nullptr;

//Pulso de passo via DDS
static inline void IRAM_ATTR stepMotor(StepperMotor &m) {
  int32_t spd = m.target_speed;
  if (spd == 0) {
    return;
  }

  if (spd > 0) {
    digitalWrite(m.dir_pin, HIGH);
  } else {
    digitalWrite(m.dir_pin, LOW);
  }

  m.acc += abs(spd);
  if (m.acc >= STEP_TIMER_FREQ_HZ) {
    m.acc -= STEP_TIMER_FREQ_HZ;
    digitalWrite(m.step_pin, HIGH);
  }
}

//ISR do timer
void IRAM_ATTR isrStepTimer() {
  digitalWrite(STEP_L, LOW);
  digitalWrite(STEP_R, LOW);
  stepMotor(motorL);
  stepMotor(motorR);
}

//Velocidade do motor
void motorSetSpeed(StepperMotor &m, float speed) {
  if (m.invert) {
    speed = -speed;
  }
  m.target_speed = (int32_t)speed;
}

//Inicializa pinos e timer
void setupMotors() {
  pinMode(STEP_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(STEP_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);

  digitalWrite(STEP_L, LOW);
  digitalWrite(STEP_R, LOW);

  stepTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(stepTimer, &isrStepTimer, true);
  timerAlarmWrite(stepTimer, 1000000 / STEP_TIMER_FREQ_HZ, true);
  timerAlarmEnable(stepTimer);
}

// ================= IMU (MPU6050) =================
//Variáveis da IMU
#define MPU_ADDR 0x68
float gyro_offset = 0.0f;
volatile float robot_angle = 0.0f;
static const float RAD2DEG = 57.2958f;

//Escrita I2C
static void imuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

//Leitura I2C
static int imuReadRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return -1;
  }
  int got = Wire.requestFrom((int)MPU_ADDR, (int)n);
  for (int i = 0; i < got; i++) {
    buf[i] = Wire.read();
  }
  return got;
}

//Inicializa IMU
static void imuInit() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(3);

  imuWriteReg(0x6B, 0x01);
  imuWriteReg(0x1A, IMU_DLPF_CFG);
  delay(100);
}

//Leitura dos sensores
void readIMU(ImuSample &s) {
  uint8_t b[14];
  if (imuReadRegs(0x3B, b, 14) != 14) {
    return;
  }

  int16_t ax_raw = (int16_t)((b[0] << 8) | b[1]);
  int16_t az_raw = (int16_t)((b[4] << 8) | b[5]);
  int16_t gy_raw = (int16_t)((b[10] << 8) | b[11]);

  const float ACC_SCALE = 9.80665f / 16384.0f;
  const float GYR_SCALE = (1.0f / 131.0f) * 0.01745329f;

  s.ax = ax_raw * ACC_SCALE;
  s.az = az_raw * ACC_SCALE;
  s.gy = gy_raw * GYR_SCALE;
}

//Ângulo pelo acelerômetro
static inline float accAngle(const ImuSample &s) {
  return atan2f(s.ax, s.az) * RAD2DEG;
}

//Calibração da IMU
void calibrateIMU() {
  Serial.println(">> Calibrando giroscopio... MANTENHA O ROBO PARADO!");

  ImuSample s = {0, 0, 0};
  float soma = 0;
  for (int i = 0; i < 200; i++) {
    readIMU(s);
    soma += s.gy;
    delay(5);
  }
  gyro_offset = soma / 200.0f;

  readIMU(s);
  robot_angle = accAngle(s);
  angleZero = robot_angle;
  Serial.printf(">> Calibracao OK! Offset giro: %.4f rad/s | angleZero: %.2f graus\n",
                gyro_offset, angleZero);
}

//Filtro complementar
bool updateComplementaryFilter(float &gyro_rate, float &dt_out) {
  static uint32_t last_us = 0;
  static ImuSample s = {0, 0, 0};

  uint32_t now = micros();
  if (last_us == 0) {
    last_us = now;
    return false;
  }
  float dt = (now - last_us) / 1e6f;
  last_us = now;

  if (dt <= 0.0f) {
    return false;
  }
  if (dt > 0.05f) {
    dt = CONTROL_PERIOD_MS / 1000.0f;
  }
  dt_out = dt;

  readIMU(s);

  gyro_rate = GYRO_SIGN * (s.gy - gyro_offset) * RAD2DEG;
  robot_angle = 0.99f * (robot_angle + gyro_rate * dt) + 0.01f * accAngle(s);
  return true;
}

//Erro angular
static inline float relAngle() {
  return robot_angle - angleZero;
}

// ================= SERIAL =================
//Controle de telemetria
QueueHandle_t xQueueTelemetry;
uint32_t pauseTelemetryUntil = 0;
volatile bool calibRequest = false;

//Leitura de comandos
static void parseSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) {
    return;
  }

  pauseTelemetryUntil = millis() + CMD_PAUSE_MS;

  if (line == "0" || line == "z") {
    calibRequest = true;
    return;
  }
  if (line == "go" || line == "g") {
    armEnabled = true;
    Serial.println(">> PID LIBERADO.");
    return;
  }
  if (line == "stop" || line == "s") {
    armEnabled = false;
    Serial.println(">> PID PARADO.");
    return;
  }

  if (line.startsWith("kp=")) {
    Kp = line.substring(3).toFloat();
  }
  if (line.startsWith("kd=")) {
    Kd = line.substring(3).toFloat();
  }
  if (line.startsWith("zero=")) {
    angleZero = line.substring(5).toFloat();
  }
}

// ================= TASKS =================
//Task de controle
void TaskControl(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

  float gyro_filt = 0.0f;
  float speed = 0.0f;
  bool armed = false;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    //Executa calibração se solicitada
    if (calibRequest) {
      armEnabled = false;
      motorSetSpeed(motorL, 0.0f);
      motorSetSpeed(motorR, 0.0f);
      calibrateIMU();
      calibRequest = false;
      xLastWakeTime = xTaskGetTickCount();
      continue;
    }

    //Atualiza leitura angular
    float gyro_rate = 0.0f;
    float dt = CONTROL_PERIOD_MS / 1000.0f;
    bool ok = updateComplementaryFilter(gyro_rate, dt);

    gyro_filt += GYRO_LPF_ALPHA * (gyro_rate - gyro_filt);
    float angle = relAngle();

    //Lógica de armar/desarmar
    bool wasArmed = armed;
    if (!armEnabled) {
      armed = false;
    } else if (fabsf(angle) > FALL_ANGLE) {
      armed = false;
    } else if (!armed && fabsf(angle) < ARM_ANGLE) {
      armed = true;
    }
    if (armed != wasArmed) {
      speed = 0.0f;
    }

    //Cálculo do PD e acionamento
    if (ok && armed) {
      float accel = constrain(Kp * angle + Kd * gyro_filt, -MAX_ACCEL, MAX_ACCEL);
      speed = constrain(speed + accel * dt, -MAX_SPEED, MAX_SPEED);

      motorSetSpeed(motorL, speed);
      motorSetSpeed(motorR, speed);
    } else {
      motorSetSpeed(motorL, 0.0f);
      motorSetSpeed(motorR, 0.0f);
    }

    //Envia dados para telemetria
    TelemetryData t = {angle, gyro_filt, speed, dt * 1e6f, armed};
    xQueueOverwrite(xQueueTelemetry, &t);
  }
}

//Task de telemetria
void TaskTelemetry(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);
  TelemetryData t;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    parseSerialCommands();
    if (millis() < pauseTelemetryUntil) {
      continue;
    }

    //Imprime dados no Serial
    if (xQueueReceive(xQueueTelemetry, &t, 0) == pdPASS) {
      const char *armado = "NAO";
      if (t.armed) {
        armado = "SIM";
      }
      Serial.printf("angulo:%.2f rate:%.1f vel/100:%.1f Kp:%.1f Kd:%.1f armado:%s loop_ms:%.2f\n",
                    t.angle, t.rate / 10.0f, t.speed / 100.0f, Kp, Kd,
                    armado, t.loop_us / 1000.0f);
    }
  }
}

//Inicialização geral
void setup() {
  Serial.begin(115200);
  delay(800);

  imuInit();
  setupMotors();

  xQueueTelemetry = xQueueCreate(1, sizeof(TelemetryData));

  xTaskCreatePinnedToCore(TaskControl,   "Control",   4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskTelemetry, "Telemetry", 4096, NULL, 1, NULL, 0);
}

//Loop inativo
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}