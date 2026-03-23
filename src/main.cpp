/**
 * @file main.cpp
 * @brief ATmega328P DAQ — Banco de Identificación de Sistemas
 * @author Atacama Dynamics
 * @version 5.2.0 — UART simplificado (datos crudos, sin máquinas de estado)
 *
 * Firmware unificado Master/Slave (ROLE_PIN PD7).
 * Cero FSM, cero PID, cero JSON. Solo DAQ + escalones.
 *
 * Master: SPI Master → ESP32, UART → Slave
 * Slave:  UART → Master
 *
 * TelemetryPacket 22 bytes (Master → ESP32 → PC)
 *
 * Comunicación UART entre Master y Slave:
 *   - Master envía valores de 2 bytes en big‑endian (uint16_t o int16_t).
 *   - Slave responde a solicitud de telemetría con 8 bytes en little‑endian.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <AS5600.h>

// ============================================================================
// CONFIGURACIÓN DE HARDWARE
// ============================================================================

#define ROLE_PIN        7    // PD7: HIGH=Master, LOW=Slave

// Motor DC (L298N)
#define EN_PIN          5    // PD5 — PWM enable
#define IN1_PIN         A3   // PC3 — dirección A
#define IN2_PIN         A2   // PC2 — dirección B

// Motor Brushless (ESC)
#define ESC_PIN         4    // PD4 — señal PWM al ESC

// Encoder de efecto Hall (motor DC)
#define ENCODER_PIN_A   2    // PD2 (INT0)
#define ENCODER_PIN_B   3    // PD3 (INT1)
#define MOTOR_GEAR_RATIO  370
#define MOTOR_ENCODER_PPR 11
#define ENCODER_PPR_WHEEL ((long)MOTOR_ENCODER_PPR * (long)MOTOR_GEAR_RATIO)
#define ENCODER_UPDATE_MS_NORMAL 10  // 100 Hz (modo compartido)
#define ENCODER_UPDATE_MS_FAST   2   // 500 Hz (modo DC exclusivo)

// AS5600 (BLDC)
#define BLDC_POLL_MS    2      // 500 Hz I2C polling
#define BLDC_CALC_MS    100    // 10 Hz cálculo Hz
#define BLDC_EMA_ALPHA  0.3f
#define BLDC_OUTLIER    512    // delta raw > esto = outlier

// SPI al ESP32 (Master solamente)
#define SPI_CS_PIN      10   // PB2 — CS del ESP32

// Intervalo de envío de telemetría (Master)
#define TELEM_INTERVAL_MS  5  // 200 Hz al ESP32

// Intervalo de solicitud de telemetría del Slave (Master)
#define SLAVE_TELEM_PUSH_MS  10  // 100 Hz request

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
#define SPI_CMD_MODE_DC     0x77
#define SPI_CMD_MODE_AC     0x78

// ============================================================================
// PROTOCOLO UART: MASTER ↔ SLAVE (datos crudos)
// ============================================================================
// Los comandos se envían como un valor de 2 bytes (big‑endian) con significado:
#define UART_REQ_TELEM       0xFFFF   // Solicitud de telemetría
#define UART_MODE_DC_VAL     0x8A00   // Cambio a modo DC
#define UART_MODE_AC_VAL     0x8B00   // Cambio a modo AC

// El Slave responde a UART_REQ_TELEM con 8 bytes en little‑endian:
// [ RPM_L, RPM_H, Hz_L, Hz_H, PWM_L, PWM_H, ESC_L, ESC_H ]

// ============================================================================
// PAQUETE DE TELEMETRÍA (22 bytes, enviado por SPI al ESP32)
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
    uint8_t  checksum;      // XOR bytes[0..20]
};

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================
static bool isMaster = false;

// --- Modo de prueba ---
enum TestMode { MODE_DC, MODE_AC };
static TestMode currentMode = MODE_DC;

// --- Estado de actuación local ---
static int16_t  appliedPWM = 0;      // PWM aplicado al DC local
static uint16_t appliedESC = ESC_NEUTRAL; // µs aplicado al ESC local

// --- Telemetría del Slave (vista desde el Master) ---
static int16_t  slaveRPM      = 0;
static uint16_t slaveHz100    = 0;
static int16_t  slavePWM      = 0;
static uint16_t slaveESC      = ESC_NEUTRAL;

// --- Filtro de reenvío: evitar saturar al Slave ---
static int16_t  lastSentDcToSlave = -999;
static uint16_t lastSentAcToSlave = 0;

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
    uint8_t interval = (currentMode == MODE_DC) ? ENCODER_UPDATE_MS_FAST : ENCODER_UPDATE_MS_NORMAL;
    if ((now - encLastTime) < interval) return;

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

static void motorSetPWM(int16_t pwm) {
    // Clamping de seguridad
    if (pwm > 255) pwm = 255;
    if (pwm < -255) pwm = -255;
    appliedPWM = pwm;
    if (pwm == 0) {
        analogWrite(EN_PIN, 0);
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, LOW);
        return;
    }

    bool forward;
    if (isMaster) {
        forward = (pwm > 0);
    } else {
        forward = (pwm < 0);  // Slave enfrentado
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
// ESC — Motor Brushless
// ============================================================================
static void escArmIfNeeded() {
    if (escArmed) return;
    esc.attach(ESC_PIN);
    esc.writeMicroseconds(1000);
    delay(2000);
    esc.writeMicroseconds(ESC_NEUTRAL);
    delay(500);
    escArmed = true;
}

static void escSetUS(uint16_t us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    appliedESC = us;
    if (!escArmed) return;
    esc.writeMicroseconds(us);
}

static void escStop() {
    appliedESC = ESC_NEUTRAL;
    if (escArmed) {
        esc.writeMicroseconds(ESC_NEUTRAL);
    }
}

// ============================================================================
// UART — COMUNICACIÓN SIMPLIFICADA (datos crudos)
// ============================================================================
// Master: envía valores de 2 bytes, solicita telemetría periódicamente.
// Slave:  recibe valores y actúa, responde a solicitud con 8 bytes.

static void uartSendVal(int16_t val) {
    Serial.write((uint8_t)((val >> 8) & 0xFF));
    Serial.write((uint8_t)(val & 0xFF));
}

static void uartProcess() {
    // Slave: leer comandos (siempre que haya al menos 2 bytes)
    if (Serial.available() >= 2) {
        uint8_t hi = Serial.read();
        uint8_t lo = Serial.read();
        int16_t val = ((uint16_t) hi << 8) | lo;

        // Detección de comandos especiales
        if (val == 0) {
            motorSetPWM(0);
            escStop();
        } else if (val == UART_MODE_DC_VAL) {
            motorSetPWM(0);
            escStop();
            currentMode = MODE_DC;
        } else if (val == UART_MODE_AC_VAL) {
            motorSetPWM(0);
            escStop();
            currentMode = MODE_AC;
        } else if (val == UART_REQ_TELEM) {
            // Responder con telemetría (8 bytes little‑endian)
            int16_t  rpm   = encoderGetRPM();
            uint16_t hz100 = bldcGetHz100();
            int16_t  pwm   = appliedPWM;
            uint16_t esc   = appliedESC;
            Serial.write((uint8_t*)&rpm,   2);
            Serial.write((uint8_t*)&hz100, 2);
            Serial.write((uint8_t*)&pwm,   2);
            Serial.write((uint8_t*)&esc,   2);
        } else if (currentMode == MODE_DC && val >= -255 && val <= 255) {
            motorSetPWM(val);
        } else if (currentMode == MODE_AC && val >= 1000 && val <= 2000) {
            escArmIfNeeded();
            escSetUS((uint16_t)val);
        }
    }
}

static void masterRequestTelem() {
    // Enviar solicitud (0xFFFF big‑endian)
    Serial.write(0xFF);
    Serial.write(0xFF);

    // Leer respuesta de 8 bytes con timeout de 5 ms
    Serial.setTimeout(5);
    uint8_t buf[8];
    if (Serial.readBytes(buf, 8) == 8) {
        slaveRPM   = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
        slaveHz100 = (uint16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
        slavePWM   = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
        slaveESC   = (uint16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8));
    }
}

// ============================================================================
// SPI — MASTER → ESP32
// ============================================================================
static uint8_t spiRxBuf[22];  // Respuesta del ESP32 (3 bytes útiles)

static void spiInit() {
    pinMode(SPI_CS_PIN, OUTPUT);
    digitalWrite(SPI_CS_PIN, HIGH);
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8);  // 2 MHz
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
}

static void spiTransact(TelemetryPacket* pkt) {
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

// ============================================================================
// PROCESAR COMANDO SPI (del ESP32)
// ============================================================================
static void processCommand(uint8_t cmd, int16_t val) {
    switch (cmd) {
        case SPI_CMD_DC_BOTH:
            if (currentMode != MODE_DC) break;
            motorSetPWM(val);
            if (val != lastSentDcToSlave) {
                uartSendVal(val);
                lastSentDcToSlave = val;
            }
            break;

        case SPI_CMD_AC_MASTER:
            if (currentMode != MODE_AC) break;
            escArmIfNeeded();
            escSetUS((uint16_t)val);
            uartSendVal(0);   // Apagar Slave
            break;

        case SPI_CMD_AC_SLAVE:
            if (currentMode != MODE_AC) break;
            escStop();
            uartSendVal((uint16_t)val);
            break;

        case SPI_CMD_AC_BOTH:
            if (currentMode != MODE_AC) break;
            escArmIfNeeded();
            escSetUS((uint16_t)val);
            if ((uint16_t)val != lastSentAcToSlave) {
                uartSendVal((uint16_t)val);
                lastSentAcToSlave = (uint16_t)val;
            }
            break;

        case SPI_CMD_STOP_ALL:
            motorSetPWM(0);
            escStop();
            lastSentDcToSlave = -999;
            lastSentAcToSlave = 0;
            uartSendVal(0);   // STOP al Slave
            break;

        case SPI_CMD_MODE_DC:
            motorSetPWM(0);
            escStop();
            lastSentDcToSlave = -999;
            lastSentAcToSlave = 0;
            currentMode = MODE_DC;
            uartSendVal(UART_MODE_DC_VAL);
            break;

        case SPI_CMD_MODE_AC:
            motorSetPWM(0);
            escStop();
            lastSentDcToSlave = -999;
            lastSentAcToSlave = 0;
            currentMode = MODE_AC;
            uartSendVal(UART_MODE_AC_VAL);
            break;

        default:
            break;
    }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    pinMode(ROLE_PIN, INPUT_PULLUP);
    isMaster = (digitalRead(ROLE_PIN) == HIGH);

    motorInit();
    encoderInit();
    bldcInit();

    if (isMaster) {
        Serial.begin(115200);
        spiInit();
    } else {
        Serial.begin(115200);
    }
}

// ============================================================================
// LOOP
// ============================================================================
static unsigned long lastTelemSend    = 0;
static unsigned long lastSlaveRequest = 0;

void loop() {
    // 1. Sensores según modo
    if (currentMode == MODE_DC) {
        encoderUpdate();
    } else {
        bldcUpdate();
    }

    // 2. UART: Slave escucha comandos, Master lee telemetría (solo si hay datos)
    if (isMaster) {
        // Master: no procesa comandos UART entrantes (ya no los usa)
        // Solo periodicamente solicita telemetría
        unsigned long now = millis();
        if ((now - lastSlaveRequest) >= SLAVE_TELEM_PUSH_MS) {
            lastSlaveRequest = now;
            masterRequestTelem();
        }
    } else {
        // Slave: siempre procesa comandos entrantes
        uartProcess();
    }

    // 3. Master: envío de telemetría al ESP32
    if (isMaster) {
        unsigned long now = millis();
        if ((now - lastTelemSend) >= TELEM_INTERVAL_MS) {
            lastTelemSend = now;

            TelemetryPacket pkt;
            pkt.startByte    = 0xAA;
            pkt.timestamp_ms = now;
            pkt.mPWM_applied = appliedPWM;
            pkt.sPWM_applied = slavePWM;
            pkt.mESC_applied = appliedESC;
            pkt.sESC_applied = slaveESC;
            pkt.mRPM         = (currentMode == MODE_DC) ? encoderGetRPM() : 0;
            pkt.sRPM         = slaveRPM;
            pkt.mHz          = (currentMode == MODE_AC) ? bldcGetHz100() : 0;
            pkt.sHz          = slaveHz100;

            spiTransact(&pkt);

            // Leer comando SPI del ESP32
            uint8_t cmd = spiRxBuf[0];
            if (cmd != SPI_CMD_NONE) {
                int16_t val = (int16_t)(((uint16_t)spiRxBuf[1] << 8) |
                              spiRxBuf[2]);
                processCommand(cmd, val);
            }
        }
    }
}