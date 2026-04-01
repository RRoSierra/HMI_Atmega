// ============================================================================
// Protocolo binario - Atacama Dynamics DAQ
// ============================================================================
// Recepción: TelemetryPacket de 22 bytes crudos
//   <B I h h H H h h H H B>  (little-endian)
// Envío: 3 bytes crudos [CMD, VAL_H, VAL_L]
// ============================================================================

pub const PKT_SIZE: usize = 22;
pub const PKT_START: u8 = 0xAA;

// Comandos (3 bytes: CMD + VAL_H + VAL_L)
pub const CMD_DC_BOTH: u8 = 0x74;
pub const CMD_AC_MASTER: u8 = 0x72;
pub const CMD_AC_SLAVE: u8 = 0x73;
pub const CMD_AC_BOTH: u8 = 0x75;
pub const CMD_STOP_ALL: u8 = 0x76;

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
}

impl TelemetryPacket {
    /// Intenta parsear un frame de 22 bytes. Retorna None si start byte o checksum fallan.
    pub fn parse(data: &[u8; PKT_SIZE]) -> Option<Self> {
        if data[0] != PKT_START {
            return None;
        }

        // Verificar checksum XOR de los primeros 21 bytes
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
