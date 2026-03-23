/**
 * @file main.cpp
 * @brief ATmega328P V2 — "Marioneta" DAQ para Identificación de Sistemas
 * @author Atacama Dynamics
 * @version 2.0.0
 *
 * Firmware minimalista unificado Master/Slave (ROLE_PIN PD7).
 * Objetivo: recibir actuación del HMI y reportar medición a máxima frecuencia.
 *
 * Arquitectura:
 *   PC (HMI.py) ↔ USB 460800 ↔ ESP32 (bypass SPI) ↔ ATmega Master ↔ ATmega Slave (UART 115200)
 *
 * Protocolo SPI (Master ↔ ESP32):
 *   TX: TelemetryPacket 22 bytes cada 5ms (200 Hz)
 *   RX: 3 bytes [CMD, VAL_H, VAL_L] del HMI
 *
 * Protocolo UART (Master ↔ Slave, 115200, full-duplex):
 *   Master→Slave: 3 bytes [CMD, HI, LO]   (comandos)
 *   Slave→Master: 10 bytes [HDR, data×8, XOR]  (telemetría push cada 10ms)
 *
 * Coordinación inversa: Master y Slave están enfrentados (180°).
 * Para moverse en la misma dirección global, Slave invierte su motor DC.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <AS5600.h>

// ═══════════════════════════════════════════════════════════════════════
// § 1. HARDWARE
// ═══════════════════════════════════════════════════════════════════════

#define ROLE_PIN     7     // PD7: HIGH = Master, LOW = Slave
#define EN_PIN       5     // PD5 — PWM enable (L298N)
#define IN1_PIN      A3    // PC3 — dirección A
#define IN2_PIN      A2    // PC2 — dirección B
#define ESC_PIN      4     // PD4 — señal PWM al ESC
#define ENCODER_A    2     // PD2 (INT0)
#define ENCODER_B    3     // PD3 (INT1)
#define SPI_CS_PIN   10    // PB2 — CS del ESP32

// ═══════════════════════════════════════════════════════════════════════
// § 2. CONSTANTES DE TIMING
// ═══════════════════════════════════════════════════════════════════════

#define ENC_CALC_MS      2     // 500 Hz cálculo RPM
#define BLDC_POLL_MS     2     // 500 Hz lectura I2C AS5600
#define BLDC_CALC_MS     100   // 10 Hz cálculo Hz
#define BLDC_EMA_ALPHA   0.3f
#define BLDC_OUTLIER     512   // delta > esto = descartado
#define TELEM_SPI_MS     5     // 200 Hz telemetría al ESP32
#define SLAVE_PUSH_MS    10    // 100 Hz telemetría Slave→Master
#define ESC_NEUTRAL      1500

// Encoder
#define ENC_PPR  ((long)11 * 370)  // 4070 pulsos/revolución

// ═══════════════════════════════════════════════════════════════════════
// § 3. PROTOCOLO SPI — COMANDOS HMI → ESP32 → ATmega (3 bytes)
// ═══════════════════════════════════════════════════════════════════════

#define CMD_NONE       0x00
#define CMD_DC_BOTH    0x74
#define CMD_AC_MASTER  0x72
#define CMD_AC_SLAVE   0x73
#define CMD_AC_BOTH    0x75
#define CMD_STOP       0x76
#define CMD_MODE_DC    0x77
#define CMD_MODE_AC    0x78

// ═══════════════════════════════════════════════════════════════════════
// § 4. PROTOCOLO UART — MASTER ↔ SLAVE
// ═══════════════════════════════════════════════════════════════════════

// Master → Slave: 3 bytes [CMD, HI, LO]
#define UCMD_DC       0xA1   // Set DC PWM (int16_t)
#define UCMD_AC       0xA2   // Set ESC µs (uint16_t)
#define UCMD_STOP     0xA3   // Stop all
#define UCMD_MODE_DC  0xA4   // Cambiar a modo DC
#define UCMD_MODE_AC  0xA5   // Cambiar a modo AC

// Slave → Master: 10 bytes [HDR, RPM_L, RPM_H, Hz_L, Hz_H, PWM_L, PWM_H, ESC_L, ESC_H, XOR]
#define UTELEM_HDR    0xBB

// ═══════════════════════════════════════════════════════════════════════
// § 5. PAQUETE DE TELEMETRÍA — 22 bytes via SPI al ESP32
// ═══════════════════════════════════════════════════════════════════════

struct __attribute__((packed)) TelemetryPacket {
    uint8_t  start;    // 0xAA
    uint32_t ts_ms;    // millis()
    int16_t  mPWM;     // Master DC PWM aplicado (-255..255)
    int16_t  sPWM;     // Slave DC PWM aplicado
    uint16_t mESC;     // Master ESC µs (1000..2000)
    uint16_t sESC;     // Slave ESC µs
    int16_t  mRPM;     // Master encoder RPM
    int16_t  sRPM;     // Slave encoder RPM
    uint16_t mHz;      // Master Hz × 100
    uint16_t sHz;      // Slave Hz × 100
    uint8_t  xor_chk;  // XOR bytes[0..20]
};

// ═══════════════════════════════════════════════════════════════════════
// § 6. VARIABLES GLOBALES
// ═══════════════════════════════════════════════════════════════════════

static bool isMaster;

enum Mode { DC, AC };
static Mode mode = DC;

// Actuación local
static int16_t  localPWM = 0;
static uint16_t localESC = ESC_NEUTRAL;

// Telemetría del Slave (vista desde Master)
static int16_t  slvRPM = 0;
static uint16_t slvHz  = 0;
static int16_t  slvPWM = 0;
static uint16_t slvESC = ESC_NEUTRAL;

// Filtro de reenvío (evita saturar UART al Slave)
static int16_t  lastFwdDC = -999;
static uint16_t lastFwdAC = 0;

// Objetos
static Servo esc;
static AS5600 as5600Sensor(&Wire);
static bool escArmed = false;

// ═══════════════════════════════════════════════════════════════════════
// § 7. ENCODER — ISR + cálculo RPM (500 Hz)
// ═══════════════════════════════════════════════════════════════════════

static volatile long encCount = 0;
static long          encPrev  = 0;
static unsigned long encT     = 0;
static float         encRPM   = 0.0f;

static void isrA() {
    encCount += (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) ? 1 : -1;
}
static void isrB() {
    encCount += (digitalRead(ENCODER_A) != digitalRead(ENCODER_B)) ? 1 : -1;
}

static void encoderInit() {
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), isrB, CHANGE);
    encT = millis();
}

static void encoderUpdate() {
    unsigned long now = millis();
    if (now - encT < ENC_CALC_MS) return;

    noInterrupts();
    long c = encCount;
    interrupts();

    long delta = c - encPrev;
    unsigned long dt = now - encT;
    encPrev = c;
    encT = now;

    if (dt > 0) {
        encRPM = ((float)delta / (float)ENC_PPR) * (60000.0f / (float)dt);
    }
}

static int16_t getRPM() { return (int16_t)encRPM; }

// ═══════════════════════════════════════════════════════════════════════
// § 8. AS5600 — BLDC Hz (500 Hz poll, 10 Hz cálculo)
// ═══════════════════════════════════════════════════════════════════════

static bool     a5ok    = false;
static uint16_t a5last  = 0;
static int32_t  a5acc   = 0;
static float    a5hz    = 0.0f;
static bool     a5first = true;
static unsigned long a5pollT = 0;
static unsigned long a5calcT = 0;

static void bldcInit() {
    Wire.begin();
    Wire.setClock(400000UL);
    as5600Sensor.begin();
    a5ok = as5600Sensor.isConnected();
    if (a5ok) {
        as5600Sensor.setZPosition(0);
        as5600Sensor.setMPosition(0);
        as5600Sensor.setMaxAngle(0);
        a5last = as5600Sensor.rawAngle();
    }
    a5pollT = a5calcT = millis();
}

static void bldcUpdate() {
    if (!a5ok) return;
    unsigned long now = millis();

    // Poll ángulo crudo
    if (now - a5pollT >= BLDC_POLL_MS) {
        a5pollT = now;
        uint16_t raw = as5600Sensor.rawAngle();
        if (!a5first) {
            int16_t d = (int16_t)raw - (int16_t)a5last;
            if (d >  2048) d -= 4096;
            if (d < -2048) d += 4096;
            if (d > -BLDC_OUTLIER && d < BLDC_OUTLIER) {
                a5acc += d;
            }
        } else {
            a5first = false;
        }
        a5last = raw;
    }

    // Calcular Hz
    if (now - a5calcT >= BLDC_CALC_MS) {
        unsigned long dt = now - a5calcT;
        a5calcT = now;
        if (dt > 0) {
            float hz = (float)a5acc / 4096.0f / ((float)dt / 1000.0f);
            if (hz < 0.0f) hz = -hz;
            a5hz = BLDC_EMA_ALPHA * hz + (1.0f - BLDC_EMA_ALPHA) * a5hz;
        }
        a5acc = 0;
    }
}

static uint16_t getHz100() { return (uint16_t)(a5hz * 100.0f); }

// ═══════════════════════════════════════════════════════════════════════
// § 9. MOTOR DC — L298N (coordinación inversa Master/Slave)
// ═══════════════════════════════════════════════════════════════════════

static void motorInit() {
    pinMode(EN_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    analogWrite(EN_PIN, 0);
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
}

static void motorSet(int16_t pwm) {
    if (pwm >  255) pwm =  255;
    if (pwm < -255) pwm = -255;
    localPWM = pwm;

    if (pwm == 0) {
        analogWrite(EN_PIN, 0);
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, LOW);
        return;
    }

    // Master: forward = pwm > 0
    // Slave:  forward = pwm < 0  (enfrentados = dirección invertida)
    bool fwd = isMaster ? (pwm > 0) : (pwm < 0);
    uint8_t spd = (uint8_t)(pwm < 0 ? -pwm : pwm);

    digitalWrite(IN1_PIN, fwd ? LOW  : HIGH);
    digitalWrite(IN2_PIN, fwd ? HIGH : LOW);
    analogWrite(EN_PIN, spd);
}

// ═══════════════════════════════════════════════════════════════════════
// § 10. ESC — Motor Brushless
// ═══════════════════════════════════════════════════════════════════════

static void escArm() {
    if (escArmed) return;
    esc.attach(ESC_PIN);
    esc.writeMicroseconds(1000);
    delay(2000);
    esc.writeMicroseconds(ESC_NEUTRAL);
    delay(500);
    escArmed = true;
}

static void escSet(uint16_t us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    localESC = us;
    if (escArmed) esc.writeMicroseconds(us);
}

static void escStop() {
    localESC = ESC_NEUTRAL;
    if (escArmed) esc.writeMicroseconds(ESC_NEUTRAL);
}

// ═══════════════════════════════════════════════════════════════════════
// § 11. UART — COMUNICACIÓN MASTER ↔ SLAVE
// ═══════════════════════════════════════════════════════════════════════

// --- Master: enviar comando de 3 bytes al Slave ---
static void uartSendCmd(uint8_t cmd, int16_t val) {
    uint8_t buf[3] = {
        cmd,
        (uint8_t)((val >> 8) & 0xFF),
        (uint8_t)(val & 0xFF)
    };
    Serial.write(buf, 3);
}

// --- Master: parser no-bloqueante de telemetría del Slave (10 bytes) ---
static uint8_t mRxBuf[9];   // 8 datos + 1 XOR
static uint8_t mRxIdx = 0;
static bool    mRxActive = false;

static void masterParseUART() {
    while (Serial.available()) {
        uint8_t b = Serial.read();
        if (!mRxActive) {
            if (b == UTELEM_HDR) {
                mRxActive = true;
                mRxIdx = 0;
            }
        } else {
            mRxBuf[mRxIdx++] = b;
            if (mRxIdx == 9) {
                mRxActive = false;
                // Verificar XOR
                uint8_t x = 0;
                for (uint8_t i = 0; i < 8; i++) x ^= mRxBuf[i];
                if (x == mRxBuf[8]) {
                    slvRPM = (int16_t)((uint16_t)mRxBuf[0] | ((uint16_t)mRxBuf[1] << 8));
                    slvHz  = (uint16_t)((uint16_t)mRxBuf[2] | ((uint16_t)mRxBuf[3] << 8));
                    slvPWM = (int16_t)((uint16_t)mRxBuf[4] | ((uint16_t)mRxBuf[5] << 8));
                    slvESC = (uint16_t)((uint16_t)mRxBuf[6] | ((uint16_t)mRxBuf[7] << 8));
                }
            }
        }
    }
}

// --- Slave: parser no-bloqueante de comandos (3 bytes) ---
static uint8_t sRxBuf[2];
static uint8_t sRxIdx = 0;
static uint8_t sRxCmd = 0;
static bool    sRxActive = false;

static void slaveParseUART() {
    while (Serial.available()) {
        uint8_t b = Serial.read();
        if (!sRxActive) {
            // Bytes de comando: 0xA1..0xA5
            if (b >= UCMD_DC && b <= UCMD_MODE_AC) {
                sRxCmd = b;
                sRxActive = true;
                sRxIdx = 0;
            }
        } else {
            sRxBuf[sRxIdx++] = b;
            if (sRxIdx == 2) {
                sRxActive = false;
                int16_t val = (int16_t)(((uint16_t)sRxBuf[0] << 8) | sRxBuf[1]);

                switch (sRxCmd) {
                    case UCMD_DC:
                        if (mode == DC) motorSet(val);
                        break;
                    case UCMD_AC:
                        if (mode == AC) {
                            escArm();
                            escSet((uint16_t)val);
                        }
                        break;
                    case UCMD_STOP:
                        motorSet(0);
                        escStop();
                        break;
                    case UCMD_MODE_DC:
                        motorSet(0);
                        escStop();
                        mode = DC;
                        break;
                    case UCMD_MODE_AC:
                        motorSet(0);
                        escStop();
                        mode = AC;
                        break;
                }
            }
        }
    }
}

// --- Slave: push telemetría periódica ---
static unsigned long slavePushT = 0;

static void slavePushTelem() {
    unsigned long now = millis();
    if (now - slavePushT < SLAVE_PUSH_MS) return;
    slavePushT = now;

    int16_t  rpm = getRPM();
    uint16_t hz  = getHz100();

    uint8_t pkt[10];
    pkt[0] = UTELEM_HDR;
    memcpy(&pkt[1], &rpm,      2);   // bytes 1-2: RPM (LE)
    memcpy(&pkt[3], &hz,       2);   // bytes 3-4: Hz×100 (LE)
    memcpy(&pkt[5], &localPWM, 2);   // bytes 5-6: PWM aplicado (LE)
    memcpy(&pkt[7], &localESC, 2);   // bytes 7-8: ESC aplicado (LE)

    uint8_t x = 0;
    for (uint8_t i = 1; i <= 8; i++) x ^= pkt[i];
    pkt[9] = x;

    Serial.write(pkt, 10);
}

// ═══════════════════════════════════════════════════════════════════════
// § 12. SPI — MASTER → ESP32
// ═══════════════════════════════════════════════════════════════════════

static uint8_t spiRx[22];

static void spiInit() {
    pinMode(SPI_CS_PIN, OUTPUT);
    digitalWrite(SPI_CS_PIN, HIGH);
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8);  // 2 MHz
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
}

static void spiSend(TelemetryPacket* pkt) {
    pkt->xor_chk = 0;
    uint8_t* raw = (uint8_t*)pkt;
    for (uint8_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) {
        pkt->xor_chk ^= raw[i];
    }

    digitalWrite(SPI_CS_PIN, LOW);
    for (uint8_t i = 0; i < sizeof(TelemetryPacket); i++) {
        spiRx[i] = SPI.transfer(raw[i]);
    }
    digitalWrite(SPI_CS_PIN, HIGH);
}

// ═══════════════════════════════════════════════════════════════════════
// § 13. PROCESADOR DE COMANDOS (Master: SPI → local + UART → Slave)
// ═══════════════════════════════════════════════════════════════════════

static void processCmd(uint8_t cmd, int16_t val) {
    switch (cmd) {
        case CMD_DC_BOTH:
            if (mode != DC) break;
            motorSet(val);
            if (val != lastFwdDC) {
                uartSendCmd(UCMD_DC, val);
                lastFwdDC = val;
            }
            break;

        case CMD_AC_MASTER:
            if (mode != AC) break;
            escArm();
            escSet((uint16_t)val);
            uartSendCmd(UCMD_STOP, 0);
            break;

        case CMD_AC_SLAVE:
            if (mode != AC) break;
            escStop();
            uartSendCmd(UCMD_AC, val);
            break;

        case CMD_AC_BOTH:
            if (mode != AC) break;
            escArm();
            escSet((uint16_t)val);
            if ((uint16_t)val != lastFwdAC) {
                uartSendCmd(UCMD_AC, val);
                lastFwdAC = (uint16_t)val;
            }
            break;

        case CMD_STOP:
            motorSet(0);
            escStop();
            lastFwdDC = -999;
            lastFwdAC = 0;
            uartSendCmd(UCMD_STOP, 0);
            break;

        case CMD_MODE_DC:
            motorSet(0);
            escStop();
            lastFwdDC = -999;
            lastFwdAC = 0;
            mode = DC;
            uartSendCmd(UCMD_MODE_DC, 0);
            break;

        case CMD_MODE_AC:
            motorSet(0);
            escStop();
            lastFwdDC = -999;
            lastFwdAC = 0;
            mode = AC;
            uartSendCmd(UCMD_MODE_AC, 0);
            break;
    }
}

// ═══════════════════════════════════════════════════════════════════════
// § 14. SETUP + LOOP
// ═══════════════════════════════════════════════════════════════════════

void setup() {
    pinMode(ROLE_PIN, INPUT_PULLUP);
    isMaster = (digitalRead(ROLE_PIN) == HIGH);

    Serial.begin(115200);
    motorInit();
    encoderInit();
    bldcInit();

    if (isMaster) spiInit();
}

static unsigned long telemT = 0;

void loop() {
    // 1. Sensores — siempre, máxima frecuencia
    encoderUpdate();
    bldcUpdate();

    if (isMaster) {
        // 2a. Leer telemetría del Slave (no-bloqueante)
        masterParseUART();

        // 2b. Enviar telemetría al ESP32 + leer comando SPI
        unsigned long now = millis();
        if (now - telemT >= TELEM_SPI_MS) {
            telemT = now;

            TelemetryPacket pkt;
            pkt.start = 0xAA;
            pkt.ts_ms = now;
            pkt.mPWM  = localPWM;
            pkt.sPWM  = slvPWM;
            pkt.mESC  = localESC;
            pkt.sESC  = slvESC;
            pkt.mRPM  = getRPM();
            pkt.sRPM  = slvRPM;
            pkt.mHz   = getHz100();
            pkt.sHz   = slvHz;

            spiSend(&pkt);

            // Procesar comando recibido del ESP32
            if (spiRx[0] != CMD_NONE) {
                int16_t val = (int16_t)(((uint16_t)spiRx[1] << 8) | spiRx[2]);
                processCmd(spiRx[0], val);
            }
        }
    } else {
        // 3. Slave: parsear comandos + empujar telemetría
        slaveParseUART();
        slavePushTelem();
    }
}
