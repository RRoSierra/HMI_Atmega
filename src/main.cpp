/**
 * @file main.cpp
 * @brief ATmega328P DAQ — Banco de Identificación de Sistemas
 * @author Atacama Dynamics
 * @version 4.1.0 — UART Watchdog Fix + Signed RPM
 *
 * Firmware unificado Master/Slave (ROLE_PIN PD7).
 * Cero FSM, cero PID, cero JSON. Solo DAQ + escalones.
 *
 * Master: SPI Master → ESP32, UART → Slave
 * Slave:  UART → Master
 *
 * TelemetryPacket 26 bytes (Master → ESP32 → PC)
 */

#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <AS5600.h>

// ============================================================================
// CONFIGURACIÓN DE HARDWARE
// ============================================================================

// Rol Master/Slave
#define ROLE_PIN        7    // PD7: HIGH=Master, LOW=Slave

// Motor DC (L298N)
#define EN_PIN          5    // PD5 — PWM enable
#define IN1_PIN         A3   // PC3 — dirección A
#define IN2_PIN         A2   // PC2 — dirección B
#define RELE_PIN         8

// Motor Brushless (ESC)
#define ESC_PIN         4    // PD4 — señal PWM al ESC

// Encoder de efecto Hall (motor DC)
#define ENCODER_PIN_A   2    // PD2 (INT0)
#define ENCODER_PIN_B   3    // PD3 (INT1)
#define MOTOR_GEAR_RATIO  370
#define MOTOR_ENCODER_PPR 11
#define ENCODER_PPR_WHEEL ((long)MOTOR_ENCODER_PPR * (long)MOTOR_GEAR_RATIO)
#define ENCODER_UPDATE_MS 10   // 100 Hz cálculo RPM

// AS5600 (BLDC)
#define BLDC_POLL_MS    2      // 500 Hz I2C polling
#define BLDC_CALC_MS    100    // 10 Hz cálculo Hz
#define BLDC_EMA_ALPHA  0.3f
#define BLDC_OUTLIER    512    // delta raw > esto = outlier

// SPI al ESP32 (Master solamente)
#define SPI_CS_PIN      10   // PB2 — CS del ESP32

// Intervalo de envío de telemetría (Master)
#define TELEM_INTERVAL_MS  15  // 100 Hz al ESP32

// Intervalo de request de telemetría al Slave
#define SLAVE_REQ_MS    15    // 100 Hz

// Keepalive: re-envío periódico de actuación al Slave
#define SLAVE_KEEPALIVE_MS 2  // 20 Hz

// ESC
#define ESC_NEUTRAL     1500

// ============================================================================
// PROTOCOLO SPI: COMANDOS QUE VIENEN DEL ESP32 (3 bytes)
// ============================================================================
#define SPI_CMD_NONE        0x00
#define SPI_CMD_DC_BOTH     0x74
#define SPI_CMD_AC_MASTER   0x72
#define SPI_CMD_AC_SLAVE    0x73
#define SPI_CMD_AC_BOTH     0x75
#define SPI_CMD_STOP_ALL    0x76
#define SPI_CMD_TOF_ARM     0x77
#define SPI_CMD_TOF_START   0x78
#define SPI_CMD_TOF_STOP    0x79
#define SPI_CMD_TOF_FIRE_SLAVE 0x7A  // Enviar pulso ESC al Slave (ToF reverso)
#define SPI_CMD_ASK_ENCODER 0x7B  // Solicitar diagnóstico AS5600 on-demand

// ============================================================================
// PROTOCOLO UART: MASTER ↔ SLAVE
// ============================================================================
#define UART_CMD_SET_DC     0x7A  // + 2 bytes BE (int16_t PWM)
#define UART_CMD_SET_AC     0x7B  // + 2 bytes BE (uint16_t µs)
#define UART_CMD_STOP       0x7C  // sin payload
#define UART_CMD_REQ_TELEM  0x7D  // sin payload → Slave responde 12 bytes
#define UART_RSP_TELEM      0x7E  // + 12 bytes LE (RPM, Hz100, PWM, ESC, mAngle, sAngle)
#define UART_CMD_ASK_ENCODER 0x7F  // sin payload → Slave responde 12 bytes diag AS5600
#define UART_RSP_DIAG       0x80  // + 12 bytes diag (MPOS, ZPOS, MAG, CONF, AGC, RAW, CONN)

// ============================================================================
// PAQUETE DE TELEMETRÍA (26 bytes, enviado por SPI al ESP32)
// ============================================================================
struct __attribute__((packed)) TelemetryPacket {
    uint8_t  startByte;     // 0xAA
    uint32_t timestamp_ms;  // millis()
    int16_t  mPWM_applied;  // PWM Master DC (-255..255)
    int16_t  sPWM_applied;  // PWM Slave DC (-255..255)
    uint16_t mESC_applied;  // Master ESC µs (1000-2000)
    uint16_t sESC_applied;  // Slave ESC µs (1000-2000)
    int16_t  mRPM;          // Master encoder RPM
    int16_t  sRPM;          // Slave encoder RPM
    uint16_t mHz;           // Master Hz * 100
    uint16_t sHz;           // Slave Hz * 100
    int16_t  mAngle;        // Master AS5600 instantaneous delta
    int16_t  sAngle;        // Slave AS5600 instantaneous delta
    uint8_t  checksum;      // XOR bytes[0..24]
};
static_assert(sizeof(TelemetryPacket) == 26, "Packet must be 26 bytes");

// ============================================================================
// PAQUETE DE DIAGNÓSTICO AS5600 (26 bytes, startByte=0xBB)
// ============================================================================
// Enviado on-demand cuando Rust solicita diagnóstico.
// Reemplaza un TelemetryPacket en el siguiente ciclo SPI.
struct __attribute__((packed)) EncoderDiagPacket {
    uint8_t  startByte;     // 0xBB
    uint32_t timestamp_ms;  // millis()
    uint16_t mpos_m;        // Master MPOS (magnet position)
    uint16_t zpos_m;        // Master ZPOS (zero position)
    uint16_t mag_m;         // Master MAG (magnet strength)
    uint8_t  agc_m;         // Master AGC (auto gain control)
    uint16_t raw_m;         // Master RAW ANGLE
    uint8_t  conn_m;        // Master AS5600 connected (1=yes, 0=no)
    uint16_t mpos_s;        // Slave MPOS
    uint16_t zpos_s;        // Slave ZPOS
    uint16_t mag_s;         // Slave MAG
    uint8_t  agc_s;         // Slave AGC
    uint16_t raw_s;         // Slave RAW ANGLE
    uint8_t  conn_s;        // Slave AS5600 connected
    uint8_t  checksum;      // XOR bytes[0..24]
};
static_assert(sizeof(EncoderDiagPacket) == 26, "Diag packet must be 26 bytes");

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================
static bool isMaster = false;

// --- Estado de actuación local ---
static int16_t  appliedPWM = 0;      // PWM aplicado al DC local
static uint16_t appliedESC = ESC_NEUTRAL; // µs aplicado al ESC local

// --- Telemetría del Slave (vista desde el Master) ---
static int16_t  slaveRPM      = 0;
static uint16_t slaveHz100    = 0;
static int16_t  slavePWM      = 0;
static uint16_t slaveESC      = ESC_NEUTRAL;
static int16_t  slaveAngleDelta = 0;

// --- Keepalive: último valor comandado al Slave ---
static int16_t  slaveCmdDC    = 0;

// --- Objetos ---
static Servo esc;
static AS5600 as5600(&Wire);

// --- ESC ---
static bool escArmed = false;

// ============================================================================
// ENCODER — ISR + cálculo RPM
// ============================================================================
static volatile long encPulses = 0;
static long  encLastPulses = 0;
static unsigned long encLastTime = 0;
static float encRPM = 0.0f;

static void encoderISR_A() {
    if (digitalRead(ENCODER_PIN_A) == digitalRead(ENCODER_PIN_B))
        encPulses++;
    else
        encPulses--;
}
//hola

static void encoderISR_B() {
    if (digitalRead(ENCODER_PIN_A) != digitalRead(ENCODER_PIN_B))
        encPulses++;
    else
        encPulses--;
}

static void encoderInit() {
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    encPulses = 0;
    encLastPulses = 0;
    encLastTime = millis();
    encRPM = 0.0f;
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), encoderISR_B, CHANGE);
}

static void encoderUpdate() {
    unsigned long now = millis();
    if ((now - encLastTime) < ENCODER_UPDATE_MS) return;

    noInterrupts();
    long cur = encPulses;
    interrupts();

    long delta = cur - encLastPulses;
    unsigned long dt = now - encLastTime;
    encLastPulses = cur;
    encLastTime = now;

    if (dt > 0 && ENCODER_PPR_WHEEL > 0) {
        encRPM = ((float)delta / (float)ENCODER_PPR_WHEEL) * (60000.0f / (float)dt);
    }
}

static int16_t encoderGetRPM() {
    // FIX: Retornar con signo para ver la inversión de giro en Python
    return (int16_t)encRPM;
}

// ============================================================================
// AS5600 — BLDC Hz
// ============================================================================
static bool     as5600Connected = false;
static uint16_t as5600LastRaw   = 0;
static int32_t  as5600AccDelta  = 0;
static float    as5600Filtered  = 0.0f;
static bool     as5600First     = true;
static int16_t  lastDelta       = 0;
static unsigned long as5600PollT = 0;
static unsigned long as5600CalcT = 0;

static void bldcInit() {
    Wire.begin();
    Wire.setClock(400000UL);
    as5600.begin();
    as5600Connected = as5600.isConnected();
    if (as5600Connected) {
        as5600.setZPosition(0);
        as5600.setMPosition(0);
        as5600.setMaxAngle(0);
        as5600LastRaw = as5600.rawAngle();
    }
    as5600AccDelta = 0;
    as5600Filtered = 0.0f;
    as5600First = true;
    as5600PollT = millis();
    as5600CalcT = millis();
}

static void bldcUpdate() {
    unsigned long now = millis();

    if (!as5600Connected) return;

    // Polling
    if ((now - as5600PollT) >= BLDC_POLL_MS) {
        as5600PollT = now;
        uint16_t raw = as5600.rawAngle();
        if (!as5600First) {
            int16_t delta = (int16_t)raw - (int16_t)as5600LastRaw;
            if (delta > 2048)  delta -= 4096;
            if (delta < -2048) delta += 4096;
            if (delta > -BLDC_OUTLIER && delta < BLDC_OUTLIER) {
                as5600AccDelta += delta;
                lastDelta = delta;
            }
        } else {
            as5600First = false;
        }
        as5600LastRaw = raw;
    }

    // Cálculo Hz
    if ((now - as5600CalcT) >= BLDC_CALC_MS) {
        unsigned long elapsed = now - as5600CalcT;
        as5600CalcT = now;
        if (elapsed > 0) {
            float rev = (float)as5600AccDelta / 4096.0f;
            float sec = (float)elapsed / 1000.0f;
            float hz  = rev / sec;
            if (hz < 0.0f) hz = -hz;
            as5600Filtered = BLDC_EMA_ALPHA * hz + (1.0f - BLDC_EMA_ALPHA) * as5600Filtered;
        }
        as5600AccDelta = 0;
    }
}

static uint16_t bldcGetHz100() {
    return (uint16_t)(as5600Filtered * 100.0f);
}

static int16_t bldcGetDeltaAngle() {
    return lastDelta;
}

// ============================================================================
// AS5600 — LECTURA DIRECTA DE REGISTROS (para diagnóstico)
// ============================================================================
// AS5600 I2C address: 0x36 (7-bit)
// Registros: ZPOS(0x00-0x01), MPOS(0x02-0x03), CONF(0x04-0x05),
//            ANGLE(0x0C-0x0D), AGC(0x1A), MAG(0x1B-0x1C)

static uint16_t as5600Read16(uint8_t reg) {
    Wire.beginTransmission(0x36);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x36, (uint8_t)2);
    if (Wire.available() >= 2) {
        uint8_t h = Wire.read();
        uint8_t l = Wire.read();
        return ((uint16_t)h << 8) | l;
    }
    return 0;
}

static uint8_t as5600Read8(uint8_t reg) {
    Wire.beginTransmission(0x36);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x36, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

// ============================================================================
// DIAGNÓSTICO AS5600 — ESTADO
// ============================================================================
static bool     diagReady   = false;

static uint16_t diagMposM = 0, diagZposM = 0, diagMagM = 0;
static uint8_t  diagAgcM  = 0;
static uint16_t diagRawM  = 0;

static uint16_t diagMposS = 0, diagZposS = 0, diagMagS = 0;
static uint8_t  diagAgcS  = 0;
static uint16_t diagRawS  = 0;
static uint8_t  diagConnM = 0, diagConnS = 0;

static bool     uartMasterWaitDiag = false;
static uint8_t  uartMasterDiagBuf[10];
static uint8_t  uartMasterDiagIdx = 0;
static unsigned long uartMasterDiagLastTime = 0;

// ============================================================================
// MOTOR DC — L298N
// ============================================================================
static void motorInit() {
    pinMode(EN_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    analogWrite(EN_PIN, 0);
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
}

/**
 * Aplica PWM con signo al motor DC local.
 * Positivo = "derecha" para Master, invertido para Slave.
 */
static void motorSetPWM(int16_t pwm) {
    appliedPWM = pwm;
    if (pwm == 0) {
        // Coast (rueda libre)
        analogWrite(EN_PIN, 0);
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, LOW);
        return;
    }

    // Determinar dirección real considerando inversión del Slave
    bool forward;
    if (isMaster) {
        forward = (pwm > 0);
    } else {
        forward = (pwm < 0);  // Slave está enfrentado → invierte
    }

    uint8_t speed = (uint8_t)(pwm < 0 ? -pwm : pwm);

    if (forward) {
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, HIGH);
    } else {
        digitalWrite(IN1_PIN, HIGH);
        digitalWrite(IN2_PIN, LOW);
    }
    analogWrite(EN_PIN, speed);
}

// ============================================================================
// ESC — Motor Brushless (non-blocking arm state machine)
// ============================================================================
static uint8_t  escArmState = 0;       // 0=idle, 1=waiting1000us, 2=waitingNeutral
static unsigned long escArmT0 = 0;

static void escArmIfNeeded() {
    if (escArmed || escArmState != 0) return;
    esc.attach(ESC_PIN);
    esc.writeMicroseconds(1000);
    escArmT0 = millis();
    escArmState = 1;
}

static void escArmUpdate() {
    if (escArmState == 0 || escArmed) return;
    unsigned long now = millis();
    if (escArmState == 1 && (now - escArmT0) >= 2000) {
        esc.writeMicroseconds(ESC_NEUTRAL);
        escArmT0 = now;
        escArmState = 2;
    } else if (escArmState == 2 && (now - escArmT0) >= 500) {
        escArmed = true;
        escArmState = 0;
    }
}

static bool escArmIsArming() {
    return escArmState != 0;
}

static void escSetUS(uint16_t us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    appliedESC = us;
    if (escArmed) {
        esc.writeMicroseconds(us);
    }
}

static void escStop() {
    appliedESC = ESC_NEUTRAL;
    if (escArmed) {
        esc.writeMicroseconds(ESC_NEUTRAL);
    }
}

// ============================================================================
// UART — COMUNICACIÓN MASTER ↔ SLAVE CON TIMEOUTS
// ============================================================================

// --- Slave: parseo de comandos entrantes del Master ---
static uint8_t  uartRxCmd = 0;
static uint16_t uartRxVal = 0;
static bool     uartRxReady = false;
static bool     uartRxWaitPayload = false;
static uint8_t  uartRxPayloadCmd = 0;
static uint8_t  uartRxBuf[8];
static uint8_t  uartRxIdx = 0;
static uint8_t  uartRxExpected = 0;
static unsigned long uartRxLastTime = 0; // Timeout Tracker Slave

// --- Master: respuesta de telemetría del Slave ---
static bool     uartMasterWaitResp = false;
static uint8_t  uartMasterRespBuf[12];
static uint8_t  uartMasterRespIdx = 0;
static uint8_t  uartMasterRespExpected = 0;
static unsigned long uartMasterRespLastTime = 0; // Timeout Tracker Master

static void uartPoll() {
    unsigned long now = millis();

    // FIX: Watchdog del Parser Master (Evita quedar bloqueado esperando un byte perdido)
    if (isMaster && uartMasterWaitResp) {
        if ((now - uartMasterRespLastTime) > 10) { // 10ms timeout
            uartMasterWaitResp = false;
        }
    }
    if (isMaster && uartMasterWaitDiag) {
        if ((now - uartMasterDiagLastTime) > 10) {
            uartMasterWaitDiag = false;
        }
    }

    // FIX: Watchdog del Parser Slave
    if (!isMaster && uartRxWaitPayload) {
        if ((now - uartRxLastTime) > 10) { // 10ms timeout
            uartRxWaitPayload = false;
        }
    }

    while (Serial.available()) {
        uint8_t b = Serial.read();

        if (isMaster) {
            // Master espera respuesta de telemetría del Slave
            if (uartMasterWaitResp) {
                uartMasterRespLastTime = now;
                uartMasterRespBuf[uartMasterRespIdx++] = b;
                
                if (uartMasterRespIdx >= uartMasterRespExpected) {
                    uartMasterWaitResp = false;
                    slaveRPM   = (int16_t)((uint16_t)uartMasterRespBuf[0] |
                                 ((uint16_t)uartMasterRespBuf[1] << 8));
                    slaveHz100 = (uint16_t)((uint16_t)uartMasterRespBuf[2] |
                                 ((uint16_t)uartMasterRespBuf[3] << 8));
                    slavePWM   = (int16_t)((uint16_t)uartMasterRespBuf[4] |
                                 ((uint16_t)uartMasterRespBuf[5] << 8));
                    slaveESC   = (uint16_t)((uint16_t)uartMasterRespBuf[6] |
                                 ((uint16_t)uartMasterRespBuf[7] << 8));
                    slaveAngleDelta = (int16_t)((uint16_t)uartMasterRespBuf[8] |
                                      ((uint16_t)uartMasterRespBuf[9] << 8));
                }
            } else if (uartMasterWaitDiag) {
                uartMasterDiagLastTime = now;
                uartMasterDiagBuf[uartMasterDiagIdx++] = b;

                if (uartMasterDiagIdx >= 10) {
                    uartMasterWaitDiag = false;
                    diagMposS = (uint16_t)((uint16_t)uartMasterDiagBuf[0] |
                                 ((uint16_t)uartMasterDiagBuf[1] << 8));
                    diagZposS = (uint16_t)((uint16_t)uartMasterDiagBuf[2] |
                                 ((uint16_t)uartMasterDiagBuf[3] << 8));
                    diagMagS  = (uint16_t)((uint16_t)uartMasterDiagBuf[4] |
                                 ((uint16_t)uartMasterDiagBuf[5] << 8));
                    diagAgcS  = uartMasterDiagBuf[6];
                    diagRawS  = (uint16_t)((uint16_t)uartMasterDiagBuf[7] |
                                 ((uint16_t)uartMasterDiagBuf[8] << 8));
                    diagConnS = uartMasterDiagBuf[9];
                    diagReady = true;
                }
            } else if (b == UART_RSP_TELEM) {
                uartMasterWaitResp = true;
                uartMasterRespIdx = 0;
                uartMasterRespExpected = 12;
                uartMasterRespLastTime = now;
            } else if (b == UART_RSP_DIAG) {
                uartMasterWaitDiag = true;
                uartMasterDiagIdx = 0;
                uartMasterDiagLastTime = now;
            }
        } else {
            // Slave: parsear comandos del Master
            if (uartRxWaitPayload) {
                uartRxLastTime = now;
                uartRxBuf[uartRxIdx++] = b;
                if (uartRxIdx >= uartRxExpected) {
                    uartRxWaitPayload = false;
                    uartRxVal = ((uint16_t)uartRxBuf[0] << 8) | uartRxBuf[1];
                    uartRxCmd = uartRxPayloadCmd;
                    uartRxReady = true;
                }
            } else {
                switch (b) {
                    case UART_CMD_SET_DC:
                    case UART_CMD_SET_AC:
                        uartRxWaitPayload = true;
                        uartRxPayloadCmd = b;
                        uartRxIdx = 0;
                        uartRxExpected = 2;
                        uartRxLastTime = now;
                        break;
                    case UART_CMD_STOP:
                        uartRxCmd = UART_CMD_STOP;
                        uartRxVal = 0;
                        uartRxReady = true;
                        break;
                    case UART_CMD_REQ_TELEM:
                        // Slave responde inmediatamente con sus datos
                        {
                            int16_t  rpm   = encoderGetRPM();
                            uint16_t hz100 = bldcGetHz100();
                            Serial.write(UART_RSP_TELEM);
                            Serial.write((uint8_t)(rpm & 0xFF));
                            Serial.write((uint8_t)((rpm >> 8) & 0xFF));
                            Serial.write((uint8_t)(hz100 & 0xFF));
                            Serial.write((uint8_t)((hz100 >> 8) & 0xFF));
                            Serial.write((uint8_t)(appliedPWM & 0xFF));
                            Serial.write((uint8_t)((appliedPWM >> 8) & 0xFF));
                            Serial.write((uint8_t)(appliedESC & 0xFF));
                            Serial.write((uint8_t)((appliedESC >> 8) & 0xFF));
                            int16_t angleDelta = bldcGetDeltaAngle();
                            Serial.write((uint8_t)(angleDelta & 0xFF));
                            Serial.write((uint8_t)((angleDelta >> 8) & 0xFF));
                            Serial.write((uint8_t)0);
                            Serial.write((uint8_t)0);
                        }
                        break;

                    case UART_CMD_ASK_ENCODER:
                        {
                            uint16_t mpos = as5600Read16(0x02);
                            uint16_t zpos = as5600Read16(0x00);
                            uint16_t mag  = as5600Read16(0x1B);
                            uint8_t  agc  = as5600Read8(0x1A);
                            uint16_t raw  = as5600Read16(0x0C);
                            uint8_t  conn = as5600.isConnected() ? 1 : 0;

                            Serial.write(UART_RSP_DIAG);
                            Serial.write((uint8_t)(mpos & 0xFF));
                            Serial.write((uint8_t)((mpos >> 8) & 0xFF));
                            Serial.write((uint8_t)(zpos & 0xFF));
                            Serial.write((uint8_t)((zpos >> 8) & 0xFF));
                            Serial.write((uint8_t)(mag & 0xFF));
                            Serial.write((uint8_t)((mag >> 8) & 0xFF));
                            Serial.write(agc);
                            Serial.write((uint8_t)(raw & 0xFF));
                            Serial.write((uint8_t)((raw >> 8) & 0xFF));
                            Serial.write(conn);
                        }
                        break;
                    default:
                        break;
                }
            }
        }
    }
}

// Master: enviar comando al Slave
static void uartSendDC(int16_t pwm) {
    uint16_t raw = (uint16_t)pwm;
    Serial.write(UART_CMD_SET_DC);
    Serial.write((uint8_t)((raw >> 8) & 0xFF));
    Serial.write((uint8_t)(raw & 0xFF));
}

static void uartSendAC(uint16_t us) {
    Serial.write(UART_CMD_SET_AC);
    Serial.write((uint8_t)((us >> 8) & 0xFF));
    Serial.write((uint8_t)(us & 0xFF));
}

static void uartSendStop() {
    Serial.write(UART_CMD_STOP);
}

static void uartRequestSlaveTelem() {
    // Si quedó atascado, resetear antes de solicitar
    uartMasterWaitResp = false;
    Serial.write(UART_CMD_REQ_TELEM);
}

// ============================================================================
// SPI — MASTER → ESP32
// ============================================================================
static uint8_t spiRxBuf[32];  // Respuesta del ESP32 (3 bytes útiles)

static void spiInit() {
    pinMode(SPI_CS_PIN, OUTPUT);
    digitalWrite(SPI_CS_PIN, HIGH);
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8);  // 2 MHz
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
}

/**
 * Envía TelemetryPacket al ESP32 y lee respuesta simultáneamente.
 * Retorna el comando SPI (byte 0) del ESP32.
 */
static void spiTransact(TelemetryPacket* pkt) {
    // Calcular checksum
    pkt->checksum = 0;
    uint8_t* raw = (uint8_t*)pkt;
    for (uint8_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) {
        pkt->checksum ^= raw[i];
    }

    digitalWrite(SPI_CS_PIN, LOW);
    for (uint8_t i = 0; i < sizeof(TelemetryPacket); i++) {
        spiRxBuf[i] = SPI.transfer(raw[i]);
    }
    digitalWrite(SPI_CS_PIN, HIGH);
}

static void spiTransactDiag(EncoderDiagPacket* dp) {
    digitalWrite(SPI_CS_PIN, LOW);
    uint8_t* raw = (uint8_t*)dp;
    for (uint8_t i = 0; i < sizeof(EncoderDiagPacket); i++) {
        spiRxBuf[i] = SPI.transfer(raw[i]);
    }
    digitalWrite(SPI_CS_PIN, HIGH);
}

// ============================================================================
// PROCESAR COMANDO SPI (del ESP32)
// ============================================================================
static void processCommand(uint8_t cmd, int16_t val) {
    switch (cmd) {
        case SPI_CMD_DC_BOTH:
            motorSetPWM(val);
            slaveCmdDC = val;         // Guardar para keepalive
            uartSendDC(val);          // Reenviar al Slave
            break;

        case SPI_CMD_AC_MASTER:
            escArmIfNeeded();
            escSetUS((uint16_t)val);
            break;

        case SPI_CMD_AC_SLAVE:
            uartSendAC((uint16_t)val);
            break;

        case SPI_CMD_AC_BOTH:
            escArmIfNeeded();
            escSetUS((uint16_t)val);
            uartSendAC((uint16_t)val);
            break;

        case SPI_CMD_STOP_ALL:
            motorSetPWM(0);
            escStop();
            slaveCmdDC = 0;           // Limpiar keepalive
            uartSendStop();
            break;

        case SPI_CMD_TOF_ARM:
            escArmIfNeeded();
            uartSendAC(ESC_NEUTRAL);  // Armar ESC del Slave también
            break;

        case SPI_CMD_TOF_START:
            escArmIfNeeded();
            escSetUS((uint16_t)val);
            break;

        case SPI_CMD_TOF_STOP:
            escStop();
            uartSendStop();  // Detener Slave también
            break;

        case SPI_CMD_TOF_FIRE_SLAVE:
            // Enviar valor ESC al Slave via UART (para ToF reverso)
            uartSendAC((uint16_t)val);
            break;

        case SPI_CMD_ASK_ENCODER:
            diagMposM = as5600Read16(0x02);
            diagZposM = as5600Read16(0x00);
            diagMagM  = as5600Read16(0x1B);
            diagAgcM  = as5600Read8(0x1A);
            diagRawM  = as5600Read16(0x0C);
            diagConnM = as5600.isConnected() ? 1 : 0;
            diagReady = false;
            uartMasterWaitDiag = false;
            Serial.write(UART_CMD_ASK_ENCODER);
            break;

        default:
            break;
    }
}

// ============================================================================
// SLAVE: PROCESAR COMANDO UART (del Master)
// ============================================================================
static void slaveProcessCommand() {
    if (!uartRxReady) return;
    uartRxReady = false;

    switch (uartRxCmd) {
        case UART_CMD_SET_DC:
            motorSetPWM((int16_t)uartRxVal);
            break;

        case UART_CMD_SET_AC:
            if (uartRxVal == 0) {
                escStop();
            } else {
                escArmIfNeeded();
                escSetUS(uartRxVal);
            }
            break;

        case UART_CMD_STOP:
            motorSetPWM(0);
            escStop();
            break;

        default:
            break;
    }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    // Determinar rol
    pinMode(ROLE_PIN, INPUT_PULLUP);
    isMaster = (digitalRead(ROLE_PIN) == HIGH);
    pinMode(RELE_PIN,OUTPUT);
    digitalWrite(RELE_PIN,HIGH);
    // Hardware común
    motorInit();
    encoderInit();
    bldcInit();

    // ESC: no armar ahora, se arma bajo demanda
    // esc no se attach hasta escArmIfNeeded()

    if (isMaster) {
        Serial.begin(115200);   // UART al Slave
        spiInit();              // SPI al ESP32
    } else {
        Serial.begin(115200);   // UART al Master
    }
}

// ============================================================================
// LOOP
// ============================================================================
static unsigned long lastTelemSend  = 0;
static unsigned long lastSlaveReq   = 0;
static unsigned long lastKeepalive  = 0;

void loop() {
    // 1. Sensores (siempre, ambos roles)
    encoderUpdate();
    bldcUpdate();
    escArmUpdate();

    // 2. UART (siempre)
    uartPoll();

    if (isMaster) {
        // =============================================
        // MASTER LOOP
        // =============================================
        unsigned long now = millis();

        // 3. Pedir telemetría al Slave periódicamente
        if ((now - lastSlaveReq) >= SLAVE_REQ_MS) {
            lastSlaveReq = now;
            uartRequestSlaveTelem();
        }

        // 3b. Keepalive: re-enviar actuación DC al Slave
        if ((now - lastKeepalive) >= SLAVE_KEEPALIVE_MS) {
            lastKeepalive = now;
            uartSendDC(slaveCmdDC);
        }

        // 4. Enviar paquete al ESP32
        if ((now - lastTelemSend) >= TELEM_INTERVAL_MS) {
            lastTelemSend = now;

            if (diagReady) {
                EncoderDiagPacket dp;
                dp.startByte    = 0xBB;
                dp.timestamp_ms = now;
                dp.mpos_m = diagMposM;
                dp.zpos_m = diagZposM;
                dp.mag_m  = diagMagM;
                dp.agc_m  = diagAgcM;
                dp.raw_m  = diagRawM;
                dp.conn_m = diagConnM;
                dp.mpos_s = diagMposS;
                dp.zpos_s = diagZposS;
                dp.mag_s  = diagMagS;
                dp.agc_s  = diagAgcS;
                dp.raw_s  = diagRawS;
                dp.conn_s = diagConnS;

                dp.checksum = 0;
                uint8_t* raw = (uint8_t*)&dp;
                for (uint8_t i = 0; i < sizeof(EncoderDiagPacket) - 1; i++) {
                    dp.checksum ^= raw[i];
                }

                spiTransactDiag(&dp);
                diagReady = false;
            } else {
                TelemetryPacket pkt;
                pkt.startByte    = 0xAA;
                pkt.timestamp_ms = now;
                pkt.mPWM_applied = appliedPWM;
                pkt.sPWM_applied = slavePWM;
                pkt.mESC_applied = appliedESC;
                pkt.sESC_applied = slaveESC;
                pkt.mRPM         = encoderGetRPM();
                pkt.sRPM         = slaveRPM;
                pkt.mHz          = bldcGetHz100();
                pkt.sHz          = slaveHz100;
                pkt.mAngle       = bldcGetDeltaAngle();
                pkt.sAngle       = slaveAngleDelta;

                spiTransact(&pkt);
            }

            uint8_t cmd = spiRxBuf[0];
            if (cmd != SPI_CMD_NONE) {
                int16_t val = (int16_t)(((uint16_t)spiRxBuf[1] << 8) |
                              spiRxBuf[2]);
                processCommand(cmd, val);
            }
        }

    } else {
        // =============================================
        // SLAVE LOOP
        // =============================================
        slaveProcessCommand();
    }
}