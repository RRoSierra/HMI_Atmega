# Protocolo de Comunicación — Banco de Identificación de Sistemas
**Atacama Dynamics · v6.0**

## Arquitectura General

```
┌──────────────┐   USB/UART    ┌──────────┐    SPI 2MHz    ┌─────────────────┐   UART 115200   ┌─────────────────┐
│  PC (Rust)   │◄────460800───►│  ESP32   │◄──────────────►│ ATmega328P      │◄───────────────►│ ATmega328P      │
│  rust_hmi    │   binario     │ (Bypass) │   full-duplex  │ MASTER (PD7=H)  │    binario      │ SLAVE  (PD7=L)  │
└──────────────┘               └──────────┘                └─────────────────┘                 └─────────────────┘
```

El **ESP32** actúa exclusivamente como **bridge/bypass binario**. No procesa, no interpreta, no filtra. Solo reenvía bytes entre USB y SPI.

---

## 1. PC → ESP32 (USB/UART 460800 baud)

El HMI de Rust envía **3 bytes crudos** por cada comando:

```
[ CMD ] [ VAL_H ] [ VAL_L ]
  1B       1B        1B      = 3 bytes total
```

- `CMD`: Byte de comando (ver tabla abajo)
- `VAL_H:VAL_L`: Valor de 16 bits, Big Endian (signo via complemento a 2)

### Tabla de Comandos (PC → ATmega Master)

| CMD        | Hex    | Payload (16-bit BE) | Descripción                          |
|------------|--------|---------------------|--------------------------------------|
| AC_MASTER  | `0x72` | µs (1000-2000)     | Aplicar ESC solo al Master           |
| AC_SLAVE   | `0x73` | µs (1000-2000)     | Aplicar ESC solo al Slave            |
| DC_BOTH    | `0x74` | PWM (-255..255)    | Aplicar PWM DC a Master **y** Slave  |
| AC_BOTH    | `0x75` | µs (1000-2000)     | Aplicar ESC a Master **y** Slave     |
| STOP_ALL   | `0x76` | 0 (ignorado)       | Detener todos los motores            |
| TOF_ARM    | `0x77` | 0 (ignorado)       | Armar ESC (secuencia no-bloqueante)  |
| TOF_START  | `0x78` | µs (1000-2000)     | Iniciar TOF: aplicar ESC             |
| TOF_STOP   | `0x79` | 0 (ignorado)       | Detener TOF: ESC a neutral           |

**Ejemplo Rust** (`encode_command`):
```rust
// Enviar PWM DC = 150 a ambos motores
let cmd = encode_command(CMD_DC_BOTH, 150);
// Resultado: [0x74, 0x00, 0x96]
```

---

## 2. ESP32 — Bypass Binario (bridge transparente)

### ESP32 recibe comando del PC (USB → SPI)
1. Lee 3 bytes del USB serial.
2. Los carga en el buffer `sendbuf[0..2]` de la siguiente transacción SPI.
3. Cuando el ATmega Master inicia un ciclo SPI, el ESP32 envía estos 3 bytes como respuesta MISO.

### ESP32 recibe telemetría del ATmega (SPI → USB)
1. Lee 26 bytes de la transacción SPI (buffer `recvbuf`).
2. Verifica `startByte == 0xAA` y checksum XOR.
3. Si es válido, reenvía los 26 bytes crudos por USB serial al PC.

**No existe procesamiento ni almacenamiento en el ESP32.**

---

## 3. ESP32 → PC (USB/UART 460800 baud)

El ESP32 reenvía al PC el **TelemetryPacket** de 26 bytes tal cual lo recibe del ATmega Master por SPI.

### Estructura del TelemetryPacket (26 bytes, Little Endian)

```
Offset  Tipo       Nombre          Descripción
──────  ─────────  ──────────────  ────────────────────────────────
  0      uint8      startByte       Siempre 0xAA
  1-4    uint32     timestamp_ms    millis() del ATmega Master
  5-6    int16      mPWM_applied    PWM DC Master (-255..255)
  7-8    int16      sPWM_applied    PWM DC Slave (-255..255)
  9-10   uint16     mESC_applied    ESC Master µs (1000-2000)
 11-12   uint16     sESC_applied    ESC Slave µs (1000-2000)
 13-14   int16      mRPM            RPM encoder Master
 15-16   int16      sRPM            RPM encoder Slave
 17-18   uint16     mHz             Hz×100 AS5600 Master
 19-20   uint16     sHz             Hz×100 AS5600 Slave
 21-22   int16      mAngle          Delta instantáneo AS5600 Master
 23-24   int16      sAngle          Delta instantáneo AS5600 Slave
 25      uint8      checksum        XOR de bytes [0..24]
```

**Rust struct format:** `<B I h h H H h h H H h h B`

**Parsing en Rust** (`protocol.rs` → `TelemetryPacket::parse`):
```rust
let timestamp_ms = u32::from_le_bytes([data[1], data[2], data[3], data[4]]);
let m_pwm = i16::from_le_bytes([data[5], data[6]]);
let s_pwm = i16::from_le_bytes([data[7], data[8]]);
let m_esc = u16::from_le_bytes([data[9], data[10]]);
let s_esc = u16::from_le_bytes([data[11], data[12]]);
let m_rpm = i16::from_le_bytes([data[13], data[14]]);
let s_rpm = i16::from_le_bytes([data[15], data[16]]);
let m_hz_100 = u16::from_le_bytes([data[17], data[18]]);
let s_hz_100 = u16::from_le_bytes([data[19], data[20]]);
let m_angle = i16::from_le_bytes([data[21], data[22]]);
let s_angle = i16::from_le_bytes([data[23], data[24]]);
```

El PC verifica checksum antes de aceptar el paquete:
```rust
let mut xor: u8 = 0;
for i in 0..PKT_SIZE - 1 {
    xor ^= data[i];
}
if xor != data[PKT_SIZE - 1] {
    return None;  // paquete corrupto, descartar
}
```

---

## 4. ATmega Master → ATmega Slave (UART 115200 baud)

Protocolo simplificado **fire & forget**. El Master envía comandos; no espera confirmación.

### Comandos Master → Slave

| Byte 0       | Hex    | Payload       | Descripción                        |
|--------------|--------|---------------|------------------------------------|
| SET_DC       | `0x7A` | 2B BE (int16) | Aplicar PWM DC al motor Slave      |
| SET_AC       | `0x7B` | 2B BE (uint16)| Aplicar ESC µs al motor Slave      |
| STOP         | `0x7C` | ninguno       | Detener todos los motores Slave    |
| REQ_TELEM    | `0x7D` | ninguno       | Solicitar telemetría del Slave     |

**Tramas de ejemplo:**
```
SET_DC +100:   [0x7A] [0x00] [0x64]     → 3 bytes
SET_AC 1700µs: [0x7B] [0x06] [0xA4]     → 3 bytes
STOP:          [0x7C]                    → 1 byte
REQ_TELEM:     [0x7D]                    → 1 byte
```

### Parser del Slave (máquina de 3 estados)
```
Estado 0 (CMD):  Lee un byte.
  - Si es 0x7A o 0x7B → guarda CMD, pasa a Estado 1
  - Si es 0x7C → ejecuta STOP inmediatamente
  - Si es 0x7D → responde con telemetría
  - Cualquier otro byte → se ignora (descarta ruido)

Estado 1 (HI): Lee byte alto del valor. Pasa a Estado 2.

Estado 2 (LO): Lee byte bajo del valor.
  Reconstruye: val = (HI << 8) | LO
  Ejecuta el comando y vuelve a Estado 0.
```

---

## 5. ATmega Slave → ATmega Master (UART 115200 baud)

El Slave responde a solicitudes del Master con su telemetría.

### Trama de Telemetría Slave → Master (13 bytes)

```
[ 0x7E ] [ RPM_L ] [ RPM_H ] [ Hz_L ] [ Hz_H ] [ PWM_L ] [ PWM_H ] [ ESC_L ] [ ESC_H ] [ mA_L ] [ mA_H ] [ sA_L ] [ sA_H ]
  header    int16 LE            uint16 LE          int16 LE            uint16 LE          int16 LE          int16 LE
           encoder RPM         AS5600 Hz×100      PWM aplicado        ESC µs aplicado    Angle Delta       Angle Delta
```

| Byte(s)  | Tipo     | Endian | Descripción                    |
|----------|----------|--------|--------------------------------|
| 0        | uint8    | —      | Header: siempre `0x7E`         |
| 1-2      | int16    | LE     | RPM del encoder Slave          |
| 3-4      | uint16   | LE     | Hz×100 del AS5600 Slave        |
| 5-6      | int16    | LE     | PWM DC aplicado localmente     |
| 7-8      | uint16   | LE     | ESC µs aplicado localmente     |
| 9-10     | int16    | LE     | Delta angle AS5600 Master      |
| 11-12    | int16    | LE     | Delta angle AS5600 Slave       |

### Parser del Master
```
Estado 0: Espera byte 0x7E. Cualquier otro byte → descarta.
Estado 1..12: Acumula 12 bytes en buffer.
Al completar 12 bytes → actualiza slaveRPM, slaveHz100, slavePWM, slaveESC, slaveAngleDelta.
Vuelve a Estado 0.
```

---

## 6. ESC Arm — Secuencia No-Bloqueante

El ESC requiere una secuencia de arming de2.5 segundos. El firmware usa una **state machine** no-bloqueante:

```
States:
  0 (IDLE):     escArmed=true, sin arming en progreso
  1 (WAIT_1000): ESC enviando 1000µs, esperando 2000ms
  2 (WAIT_NEUTRAL): ESC enviando 1500µs, esperando 500ms

Transiciones:
  IDLE → WAIT_1000:    escArmIfNeeded() llamado
  WAIT_1000 → WAIT_NEUTRAL: 2000ms transcurridos
  WAIT_NEUTRAL → IDLE: 500ms transcurridos, escArmed=true
```

**Importante**: Durante el arming (state 1 o 2), `escSetUS()` NO aplica valores al ESC pero SÍ almacena en `appliedESC`. Una vez completado el arming, el último valor aplicado se envía al ESC.

---

## 7. Diagrama de Flujo Temporal

```
PC (Rust)            ESP32 (Bypass)         ATmega MASTER            ATmega SLAVE
    │                     │                      │                        │
    │─── [0x74 0x00 0x96] ──►│                   │                        │
    │    (CMD: DC=150)    │                      │                        │
    │                     │──── SPI (3B cmd) ────►│                        │
    │                     │                      │── motorSetPWM(150)     │
    │                     │                      │── [0x7A 0x00 0x96] ───►│
    │                     │                      │   (si val cambió)      │── motorSetPWM(150)
    │                     │                      │                        │
    │                     │                      │◄── [0x7D] ─────────────│  cada 15ms
    │                     │                      │── solicitar telem      │
    │                     │                      │                        │
    │                     │                      │◄── [0x7E + 12B telem] ─│  responde telem
    │                     │                      │                        │
    │                     │◄── SPI (26B pkt) ────│   cada 15ms:           │
    │                     │   TelemetryPacket     │   spiTransact()       │
    │◄── [26B raw] ───────│                      │                        │
    │   (bypass directo)  │                      │                        │
    │                     │                      │                        │
    │── parse & display   │                      │                        │
```

---

## 8. Resumen de Baudrates y Frecuencias

| Enlace                  | Protocolo | Baudrate/Clock | Frecuencia datos |
|-------------------------|-----------|----------------|------------------|
| PC ↔ ESP32              | UART      | 460800 baud    | Bajo demanda (cmd) / ~67 Hz (telem) |
| ESP32 ↔ ATmega Master   | SPI       | 2 MHz          | ~67 Hz (cada 15ms) |
| ATmega Master ↔ Slave   | UART      | 115200 baud    | Cmds: bajo demanda / Telem: ~67 Hz |

---

## 9. Archivos Fuente

| Archivo | Plataforma | Función |
|---------|------------|---------|
| `rust_hmi/src/protocol.rs` | PC (Rust) | HMI: protocolo, parsing, comandos |
| `rust_hmi/src/app.rs` | PC (Rust) | HMI: interfaz gráfica egui |
| `rust_hmi/src/serial_comm.rs` | PC (Rust) | HMI: comunicación serial |
| `ATmega328- Telemetria/src/main.cpp` | ESP32 (PlatformIO) | Bridge SPI↔UART binario transparente |
| `HMI_Atmega/src/main.cpp` | ATmega328P (PlatformIO) | Firmware unificado Master/Slave (selección por PD7) |
