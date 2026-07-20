// ============================================================================
// Protocolo binario - Atacama Dynamics DAQ
// ============================================================================
// Recepción: TelemetryPacket de 26 bytes crudos
//   <B I h h H H h h h h h h B>  (little-endian)
//   [0]startByte=0xAA(1), [1-4]timestamp(4), [5-6]m_pwm(2), [7-8]s_pwm(2),
//   [9-10]m_esc(2), [11-12]s_esc(2), [13-14]m_rpm(2), [15-16]s_rpm(2),
//   [17-18]m_hz(2), [19-20]s_hz(2), [21-22]m_angle(2), [23-24]s_angle(2),
//   [25]checksum(1) = 26 bytes
// Envío: 3 bytes crudos [CMD, VAL_H, VAL_L]
// ============================================================================

pub const PKT_SIZE: usize = 26;
pub const PKT_START: u8 = 0xAA;
pub const DIAG_START: u8 = 0xBB;

pub const CMD_DC_BOTH: u8 = 0x74;
pub const CMD_AC_MASTER: u8 = 0x72;
pub const CMD_AC_SLAVE: u8 = 0x73;
pub const CMD_AC_BOTH: u8 = 0x75;
pub const CMD_STOP_ALL: u8 = 0x76;
pub const CMD_TOF_ARM: u8 = 0x77;
pub const CMD_TOF_START: u8 = 0x78;
pub const CMD_TOF_STOP: u8 = 0x79;
pub const CMD_TOF_FIRE_SLAVE: u8 = 0x7A;  // Enviar pulso ESC al Slave (ToF reverso)
pub const CMD_ASK_ENCODER: u8 = 0x7B;     // Diagnóstico AS5600 on-demand
pub const CMD_TOF_DETACH: u8 = 0x7C;      // Detach ESC receptor (val=0:local, 1:remoto)
pub const CMD_TOF_ATTACH: u8 = 0x7D;      // Attach ESC receptor  (val=0:local, 1:remoto)

#[derive(Debug, Clone)]
pub struct TelemetryPacket {
    pub timestamp_ms: u32,
    pub m_pwm: i16,
    pub s_pwm: i16,
    pub m_esc: u16,
    pub s_esc: u16,
    pub m_rpm: i16,
    pub s_rpm: i16,
    pub m_hz: f64,
    pub s_hz: f64,
    pub m_angle: i16,
    pub s_angle: i16,
}

impl TelemetryPacket {
    /// Intenta parsear un frame de 26 bytes. Retorna None si start byte o checksum fallan.
    pub fn parse(data: &[u8; PKT_SIZE]) -> Option<Self> {
        if data[0] != PKT_START {
            return None;
        }

        // Verificar checksum XOR de los primeros 25 bytes
        let mut xor: u8 = 0;
        for i in 0..PKT_SIZE - 1 {
            xor ^= data[i];
        }
        if xor != data[PKT_SIZE - 1] {
            return None;
        }

        // Desempaquetar campos (little-endian)
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

        Some(TelemetryPacket {
            timestamp_ms,
            m_pwm,
            s_pwm,
            m_esc,
            s_esc,
            m_rpm,
            s_rpm,
            m_hz: m_hz_100 as f64 / 100.0,
            s_hz: s_hz_100 as f64 / 100.0,
            m_angle,
            s_angle,
        })
    }
}

/// Codifica un comando de 3 bytes: [CMD, VAL_HIGH, VAL_LOW]
pub fn encode_command(cmd: u8, val: i16) -> [u8; 3] {
    let raw = if val >= 0 {
        val as u16
    } else {
        (val as i32 + 0x10000) as u16
    };
    [cmd, (raw >> 8) as u8, (raw & 0xFF) as u8]
}

#[derive(Debug, Clone)]
pub struct DiagPacket {
    pub timestamp_ms: u32,
    pub mpos_m: u16,
    pub zpos_m: u16,
    pub mag_m: u16,
    pub agc_m: u8,
    pub raw_m: u16,
    pub conn_m: u8,
    pub mpos_s: u16,
    pub zpos_s: u16,
    pub mag_s: u16,
    pub agc_s: u8,
    pub raw_s: u16,
    pub conn_s: u8,
}

impl DiagPacket {
    pub fn parse(data: &[u8; PKT_SIZE]) -> Option<Self> {
        if data[0] != DIAG_START {
            return None;
        }

        let mut xor: u8 = 0;
        for i in 0..PKT_SIZE - 1 {
            xor ^= data[i];
        }
        if xor != data[PKT_SIZE - 1] {
            return None;
        }

        Some(DiagPacket {
            timestamp_ms: u32::from_le_bytes([data[1], data[2], data[3], data[4]]),
            mpos_m: u16::from_le_bytes([data[5], data[6]]),
            zpos_m: u16::from_le_bytes([data[7], data[8]]),
            mag_m: u16::from_le_bytes([data[9], data[10]]),
            agc_m: data[11],
            raw_m: u16::from_le_bytes([data[12], data[13]]),
            conn_m: data[14],
            mpos_s: u16::from_le_bytes([data[15], data[16]]),
            zpos_s: u16::from_le_bytes([data[17], data[18]]),
            mag_s: u16::from_le_bytes([data[19], data[20]]),
            agc_s: data[21],
            raw_s: u16::from_le_bytes([data[22], data[23]]),
            conn_s: data[24],
        })
    }
}
