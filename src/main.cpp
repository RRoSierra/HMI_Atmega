/**
 * @file main.cpp
 * @brief ATmega328P DAQ — Banco de Identificación de Sistemas
 * @author Atacama Dynamics
 * @version 4.2.0 — Protocolo UART Robusto (StartByte + Checksum)
 *
 * Firmware unificado Master/Slave (ROLE_PIN PD7).
 * Cero FSM, cero PID, cero JSON. Solo DAQ + escalones.
 *
 * Master: SPI Master → ESP32, UART → Slave
 * Slave:  UART → Master
 *
 * TelemetryPacket 22 bytes (Master → ESP32 → PC)
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

// Intervalos
#define TELEM_INTERVAL_MS  10  // 100 Hz al ESP32
#define SLAVE_REQ_MS       10  // 100 Hz petición al Slave

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

// ============================================================================
// PROTOCOLO UART ROBUSTO: MASTER ↔ SLAVE
// ============================================================================
#define UART_START_M2S      0xBB  // Header Master a Slave
#define UART_START_S2M      0xCC  // Header Slave a Master

#define UART_CMD_SET_DC     0x7A
#define UART_CMD_SET_AC     0x7B
#define UART_CMD_STOP       0x7C
#define UART_CMD_REQ_TELEM  0x7D

// ============================================================================
// PAQUETE DE TELEMETRÍA SPI (22 bytes)
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

// Estado de actuación local
static int16_t  appliedPWM = 0;      
static uint16_t appliedESC = ESC_NEUTRAL; 

// Telemetría del Slave (vista desde el Master)
static int16_t  slaveRPM      = 0;
static uint16_t slaveHz100    = 0;
static int16_t  slavePWM      = 0;
static uint16_t slaveESC      = ESC_NEUTRAL;

// Objetos
static Servo esc;
static AS5600 as5600(&Wire);
static bool escArmed = false;

// ============================================================================
// ENCODER — ISR + cálculo RPM
// ============================================================================
static volatile long encPulses = 0;
static long  encLastPulses = 0;
static unsigned long encLastTime = 0;
static float encRPM = 0.0f;

static void encoderISR_A() {
    if (digitalRead(ENCODER_PIN_A) == digitalRead(ENCODER_PIN_B)) encPulses++;
    else encPulses--;
}

static void encoderISR_B() {
    if (digitalRead(ENCODER_PIN_A) != digitalRead(ENCODER_PIN_B)) encPulses++;
    else encPulses--;
}

static void encoderInit() {
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), encoderISR_B, CHANGE);
    encLastTime = millis();
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
    return (int16_t)encRPM; // Conserva el signo para visualizar dirección
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
}

static void bldcUpdate() {
    unsigned long now = millis();
    if (!as5600Connected) return;

    if ((now - as5600PollT) >= BLDC_POLL_MS) {
        as5600PollT = now;
        uint16_t raw = as5600.rawAngle();
        if (!as5600First) {
            int16_t delta = (int16_t)raw - (int16_t)as5600LastRaw;
            if (delta > 2048)  delta -= 4096;
            if (delta < -2048) delta += 4096;
            if (delta > -BLDC_OUTLIER && delta < BLDC_OUTLIER) as5600AccDelta += delta;
        } else {
            as5600First = false;
        }
        as5600LastRaw = raw;
    }

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
// ACTUADORES
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
    appliedPWM = pwm;
    if (pwm == 0) {
        analogWrite(EN_PIN, 0);
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, LOW);
        return;
    }

    bool forward = isMaster ? (pwm > 0) : (pwm < 0); // Slave invierte dirección física
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
    if (escArmed) esc.writeMicroseconds(us);
}

static void escStop() {
    appliedESC = ESC_NEUTRAL;
    if (escArmed) esc.writeMicroseconds(ESC_NEUTRAL);
}

// ============================================================================
// NUEVO PARSER UART ROBUSTO (Elimina errores de trama y ruido)
// ============================================================================

// Variables del parser del Master
static uint8_t mRxState = 0;
static uint8_t mRxBuf[10];

// Variables del parser del Slave
static uint8_t sRxState = 0;
static uint8_t sRxCmd, sRxH, sRxL;

// Banderas de ejecución del Slave
static bool    uartRxReady = false;
static uint8_t uartRxCmd   = 0;
static int16_t uartRxVal   = 0;

static void uartPoll() {
    while (Serial.available()) {
        uint8_t b = Serial.read();

        if (isMaster) {
            // Master escucha al Slave (Paquete de 10 bytes)
            // [0xCC] [RPM_L] [RPM_H] [Hz_L] [Hz_H] [PWM_L] [PWM_H] [ESC_L] [ESC_H] [CHECKSUM]
            if (mRxState == 0) {
                if (b == UART_START_S2M) {
                    mRxBuf[0] = b;
                    mRxState = 1;
                }
            } else {
                mRxBuf[mRxState++] = b;
                if (mRxState == 10) {
                    uint8_t chk = 0;
                    for (int i = 0; i < 9; i++) chk ^= mRxBuf[i];
                    
                    if (chk == mRxBuf[9]) { // Checksum Valido!
                        slaveRPM   = (int16_t)((uint16_t)mRxBuf[1] | ((uint16_t)mRxBuf[2] << 8));
                        slaveHz100 = (uint16_t)((uint16_t)mRxBuf[3] | ((uint16_t)mRxBuf[4] << 8));
                        slavePWM   = (int16_t)((uint16_t)mRxBuf[5] | ((uint16_t)mRxBuf[6] << 8));
                        slaveESC   = (uint16_t)((uint16_t)mRxBuf[7] | ((uint16_t)mRxBuf[8] << 8));
                    }
                    mRxState = 0;
                }
            }
        } else {
            // Slave escucha al Master (Paquete de 5 bytes)
            // [0xBB] [CMD] [VAL_H] [VAL_L] [CHECKSUM]
            if (sRxState == 0) {
                if (b == UART_START_M2S) sRxState = 1;
            } else if (sRxState == 1) {
                sRxCmd = b; sRxState = 2;
            } else if (sRxState == 2) {
                sRxH = b;   sRxState = 3;
            } else if (sRxState == 3) {
                sRxL = b;   sRxState = 4;
            } else if (sRxState == 4) {
                uint8_t chk = UART_START_M2S ^ sRxCmd ^ sRxH ^ sRxL;
                if (chk == b) { // Checksum Valido!
                    if (sRxCmd == UART_CMD_REQ_TELEM) {
                        // Construir y enviar respuesta inmediatamente
                        int16_t  rpm   = encoderGetRPM();
                        uint16_t hz100 = bldcGetHz100();
                        
                        uint8_t txBuf[10];
                        txBuf[0] = UART_START_S2M;
                        txBuf[1] = rpm & 0xFF;        txBuf[2] = (rpm >> 8) & 0xFF;
                        txBuf[3] = hz100 & 0xFF;      txBuf[4] = (hz100 >> 8) & 0xFF;
                        txBuf[5] = appliedPWM & 0xFF; txBuf[6] = (appliedPWM >> 8) & 0xFF;
                        txBuf[7] = appliedESC & 0xFF; txBuf[8] = (appliedESC >> 8) & 0xFF;
                        
                        uint8_t txChk = 0;
                        for(int i=0; i<9; i++) txChk ^= txBuf[i];
                        txBuf[9] = txChk;
                        
                        Serial.write(txBuf, 10);
                    } else {
                        // Es un comando de movimiento, agendar para procesar
                        uartRxCmd = sRxCmd;
                        uartRxVal = (int16_t)((sRxH << 8) | sRxL);
                        uartRxReady = true;
                    }
                }
                sRxState = 0;
            }
        }
    }
}

// Master: enviar comando estructurado al Slave
static void uartSendCommand(uint8_t cmd, int16_t val) {
    uint16_t raw = (uint16_t)val;
    uint8_t h = (raw >> 8) & 0xFF;
    uint8_t l = raw & 0xFF;
    uint8_t chk = UART_START_M2S ^ cmd ^ h ^ l;
    
    Serial.write(UART_START_M2S);
    Serial.write(cmd);
    Serial.write(h);
    Serial.write(l);
    Serial.write(chk);
}

static void uartSendDC(int16_t pwm) { uartSendCommand(UART_CMD_SET_DC, pwm); }
static void uartSendAC(uint16_t us) { uartSendCommand(UART_CMD_SET_AC, us); }
static void uartSendStop()          { uartSendCommand(UART_CMD_STOP, 0); }
static void uartRequestSlaveTelem() { uartSendCommand(UART_CMD_REQ_TELEM, 0); }

// ============================================================================
// SPI — MASTER → ESP32
// ============================================================================
static uint8_t spiRxBuf[22];

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
// PROCESAR COMANDO SPI (Master evalúa y envía al Slave)
// ============================================================================
static void processCommand(uint8_t cmd, int16_t val) {
    switch (cmd) {
        case SPI_CMD_DC_BOTH:
            motorSetPWM(val);
            uartSendDC(val);
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
            uartSendStop();
            break;
    }
}

// ============================================================================
// SLAVE: PROCESAR COMANDO UART
// ============================================================================
static void slaveProcessCommand() {
    if (!uartRxReady) return;
    uartRxReady = false;

    switch (uartRxCmd) {
        case UART_CMD_SET_DC:
            motorSetPWM(uartRxVal);
            break;
        case UART_CMD_SET_AC:
            if (uartRxVal == 0) {
                escStop();
            } else {
                escArmIfNeeded();
                escSetUS((uint16_t)uartRxVal);
            }
            break;
        case UART_CMD_STOP:
            motorSetPWM(0);
            escStop();
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

    Serial.begin(115200);
    if (isMaster) {
        spiInit();
    }
}

// ============================================================================
// LOOP
// ============================================================================
static unsigned long lastTelemSend  = 0;
static unsigned long lastSlaveReq   = 0;

void loop() {
    encoderUpdate();
    bldcUpdate();
    uartPoll();

    if (isMaster) {
        unsigned long now = millis();

        // 3. Pedir telemetría al Slave periódicamente
        if ((now - lastSlaveReq) >= SLAVE_REQ_MS) {
            lastSlaveReq = now;
            uartRequestSlaveTelem();
        }

        // 4. Enviar TelemetryPacket al ESP32
        if ((now - lastTelemSend) >= TELEM_INTERVAL_MS) {
            lastTelemSend = now;

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

            spiTransact(&pkt);

            uint8_t cmd = spiRxBuf[0];
            if (cmd != SPI_CMD_NONE) {
                int16_t val = (int16_t)(((uint16_t)spiRxBuf[1] << 8) | spiRxBuf[2]);
                processCommand(cmd, val);
            }
        }
    } else {
        slaveProcessCommand();
    }
}