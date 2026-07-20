// ============================================================================
// Banco de Identificación de Sistemas - HMI Desktop v4.0 (Rust/egui)
// Atacama Dynamics
//
// Equivalente funcional del HMI en Python/Tkinter, usando egui para
// renderizado inmediato con mayor fluidez en gráficos en tiempo real.
// ============================================================================

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use eframe::egui;
use egui::{Color32, RichText, Rounding, Stroke};
use egui_plot::{Legend, Line, LineStyle, Plot, PlotBounds, PlotPoints, VLine};

use crate::protocol::*;
use crate::serial_comm::*;

// ============================================================================
// PALETA DE COLORES — Modern Light Flat (idéntica al HMI Python)
// ============================================================================
const BG_MAIN: Color32 = Color32::from_rgb(0xFF, 0xFF, 0xFF);
const BG_PANEL: Color32 = Color32::from_rgb(0xF8, 0xFA, 0xFC);
const FG_TEXT: Color32 = Color32::from_rgb(0x1E, 0x29, 0x3B);
const FG_DIM: Color32 = Color32::from_rgb(0x64, 0x74, 0x8B);
const C_BORDER: Color32 = Color32::from_rgb(0xE2, 0xE8, 0xF0);
const C_BLUE: Color32 = Color32::from_rgb(0x3B, 0x82, 0xF6);
const C_GREEN: Color32 = Color32::from_rgb(0x10, 0xB9, 0x81);
const C_RED: Color32 = Color32::from_rgb(0xEF, 0x44, 0x44);
const C_ORANGE: Color32 = Color32::from_rgb(0xF9, 0x73, 0x16);
const C_CYAN: Color32 = Color32::from_rgb(0x06, 0xB6, 0xD4);
const C_YELLOW: Color32 = Color32::from_rgb(0xFB, 0xBF, 0x24);

const LIVE_BUFFER_SIZE: usize = 6000;
const SIDE_PANEL_WIDTH: f32 = 350.0;

// ============================================================================
// ToF CALIBRATION CONSTANTS
// ============================================================================
const TOF_ARM_DURATION_MS: u64 = 2000;
const TOF_SILENCE_DURATION_MS: u64 = 1000;
const TOF_PULSE_DURATION_MS: u64 = 50;
const TOF_NEUTRAL_ESC: i16 = 1500;
const TOF_LISTEN_TIMEOUT_MS: u64 = 3000;
const TOF_MIN_DETECT_MS: u64 = 0;
const MIN_BASELINE_SAMPLES: usize = 3;
const TOF_BUFFER_SIZE: usize = 1000;

// ============================================================================
// ToF STATE COLORS
// ============================================================================
const C_IDLE: Color32 = Color32::from_rgb(0x94, 0xA3, 0xB8);
const C_ARMING: Color32 = Color32::from_rgb(0xF9, 0x73, 0x16);
const C_SILENCE: Color32 = Color32::from_rgb(0xFB, 0xBF, 0x24);
const C_FIRING: Color32 = Color32::from_rgb(0xEF, 0x44, 0x44);
const C_LISTENING: Color32 = Color32::from_rgb(0x10, 0xB9, 0x81);
const C_DONE: Color32 = Color32::from_rgb(0x3B, 0x82, 0xF6);

// ============================================================================
// TIPOS
// ============================================================================
#[derive(Clone, Copy, PartialEq)]
enum AcTarget {
    Both,
    Master,
    Slave,
}

impl AcTarget {
    fn label(&self) -> &'static str {
        match self {
            AcTarget::Both => "Ambos",
            AcTarget::Master => "Master",
            AcTarget::Slave => "Slave",
        }
    }

    fn cmd(&self) -> u8 {
        match self {
            AcTarget::Both => CMD_AC_BOTH,
            AcTarget::Master => CMD_AC_MASTER,
            AcTarget::Slave => CMD_AC_SLAVE,
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Tab {
    Control,
    TofCalibration,
}

impl Tab {
    fn label(&self) -> &'static str {
        match self {
            Tab::Control => "Control",
            Tab::TofCalibration => "ToF Calibration",
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
enum TofState {
    Idle,
    Arming,
    Silence,
    Firing,
    Listening,
    Done,
}

impl TofState {
    fn label(&self) -> &'static str {
        match self {
            TofState::Idle => "Idle",
            TofState::Arming => "Arming",
            TofState::Silence => "Silence",
            TofState::Firing => "Firing",
            TofState::Listening => "Listening",
            TofState::Done => "Done",
        }
    }

    fn color(&self) -> Color32 {
        match self {
            TofState::Idle => C_IDLE,
            TofState::Arming => C_ARMING,
            TofState::Silence => C_SILENCE,
            TofState::Firing => C_FIRING,
            TofState::Listening => C_LISTENING,
            TofState::Done => C_DONE,
        }
    }
}

#[derive(Clone)]
struct TofResult {
    timestamp: String,
    target_hz: f64,
    tof_ms: f64,
    wavespeed: f64,
}

#[derive(Clone, Copy, PartialEq)]
enum TofShooter {
    Master,
    Slave,
}

// ============================================================================
// ESTADO PRINCIPAL DE LA APLICACIÓN
// ============================================================================
pub struct SysIdApp {
    // --- Serial ---
    available_ports: Vec<String>,
    selected_port_idx: usize,
    serial: Option<SerialConnection>,

    // --- Control DC ---
    dc_value: i32,
    dc_entry: String,
    dc_active: bool,

    // --- Control AC ---
    ac_value: i32,
    ac_entry: String,
    ac_active: bool,
    ac_target: AcTarget,

    // --- Grabación CSV ---
    is_recording: bool,
    data_log: Vec<TelemetryPacket>,

    // --- Telemetría ---
    last_packet: Option<TelemetryPacket>,
    pkt_count: u64,
    pkt_errors: u64,
    connection_ok: bool,
    last_rx_time: Instant,

    // --- Buffers circulares para gráficos en vivo ---
    live_times: VecDeque<f64>,
    live_m_pwm: VecDeque<f64>,
    live_s_pwm: VecDeque<f64>,
    live_m_esc: VecDeque<f64>,
    live_s_esc: VecDeque<f64>,
    live_m_rpm: VecDeque<f64>,
    live_s_rpm: VecDeque<f64>,
    live_m_hz: VecDeque<f64>,
    live_s_hz: VecDeque<f64>,
    live_t0_ms: Option<u32>,

    // --- UI ---
    status_text: String,
    status_color: Color32,
    warning_text: String,
    message: Option<(String, String)>, // (título, cuerpo)

    // --- Logo ---
    logo_texture: Option<egui::TextureHandle>,
    logo_loaded: bool,

    // --- Cierre diferido ---
    close_after: Option<Instant>,

    // --- Tabs ---
    active_tab: Tab,

    // --- ToF Calibration ---
    tof_state: TofState,
    tof_target_hz: f64,
    tof_entry_hz: String,
    tof_threshold: f64,
    tof_rope_length: f64,
    tof_rope_entry: String,
    tof_coeff_a: f64,
    tof_coeff_b: f64,
    tof_a_entry: String,
    tof_b_entry: String,
    tof_arm_time: Option<Instant>,
    tof_armed: bool,
    tof_t0_ms: Option<u32>,
    tof_t1_ms: Option<u32>,
    tof_tof_ms: f64,
    tof_wavespeed: f64,
    tof_wave_detected: bool,
    tof_baseline: f64,
    tof_baseline_shooter: f64,
    tof_baseline_samples: VecDeque<f64>,
    tof_baseline_shooter_samples: VecDeque<f64>,
    tof_history: Vec<TofResult>,
    tof_times: VecDeque<f64>,
    tof_m_angle: VecDeque<f64>,
    tof_s_angle: VecDeque<f64>,
    tof_t0_line: Option<f64>,
    tof_t1_line: Option<f64>,
    tof_t1_sent: Option<Instant>,

    tof_error_msg: Option<String>,

    // --- ToF Shooter Selection ---
    tof_shooter: TofShooter,
    tof_pulse_us: i16,

    tof_firing_time: Option<Instant>,

    // --- AS5600 Diagnostic ---
    diag_data: Option<DiagPacket>,
}

// ============================================================================
// CONSTRUCTOR
// ============================================================================
impl SysIdApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Configurar estilo claro
        let mut visuals = egui::Visuals::light();
        visuals.panel_fill = BG_MAIN;
        visuals.window_fill = BG_MAIN;
        visuals.extreme_bg_color = BG_PANEL;
        visuals.widgets.noninteractive.bg_fill = BG_PANEL;
        visuals.widgets.noninteractive.fg_stroke = Stroke::new(1.0f32, FG_TEXT);
        visuals.widgets.inactive.bg_fill = Color32::from_rgb(0xF1, 0xF5, 0xF9);
        visuals.widgets.hovered.bg_fill = Color32::from_rgb(0xE2, 0xE8, 0xF0);
        cc.egui_ctx.set_visuals(visuals);

        let ports = list_ports();
        let preferred_idx = ports
            .iter()
            .position(|p| p.contains("ttyUSB"))
            .unwrap_or(0);

        SysIdApp {
            available_ports: ports,
            selected_port_idx: preferred_idx,
            serial: None,

            dc_value: 0,
            dc_entry: "0".to_string(),
            dc_active: false,

            ac_value: 1500,
            ac_entry: "1500".to_string(),
            ac_active: false,
            ac_target: AcTarget::Both,

            is_recording: false,
            data_log: Vec::new(),

            last_packet: None,
            pkt_count: 0,
            pkt_errors: 0,
            connection_ok: false,
            last_rx_time: Instant::now(),

            live_times: VecDeque::with_capacity(LIVE_BUFFER_SIZE),
            live_m_pwm: VecDeque::with_capacity(LIVE_BUFFER_SIZE),
            live_s_pwm: VecDeque::with_capacity(LIVE_BUFFER_SIZE),
            live_m_esc: VecDeque::with_capacity(LIVE_BUFFER_SIZE),
            live_s_esc: VecDeque::with_capacity(LIVE_BUFFER_SIZE),
            live_m_rpm: VecDeque::with_capacity(LIVE_BUFFER_SIZE),
            live_s_rpm: VecDeque::with_capacity(LIVE_BUFFER_SIZE),
            live_m_hz: VecDeque::with_capacity(LIVE_BUFFER_SIZE),
            live_s_hz: VecDeque::with_capacity(LIVE_BUFFER_SIZE),
            live_t0_ms: None,

            status_text: "DESCONECTADO".to_string(),
            status_color: C_RED,
            warning_text: String::new(),
            message: None,

            logo_texture: None,
            logo_loaded: false,

            close_after: None,

            active_tab: Tab::Control,

            tof_state: TofState::Idle,
            tof_target_hz: 50.0,
            tof_entry_hz: "50".to_string(),
            tof_threshold: 10.0,
            tof_rope_length: 1.0,
            tof_rope_entry: "1.0".to_string(),
            tof_coeff_a: 1.0,
            tof_coeff_b: 0.0,
            tof_a_entry: "1.0".to_string(),
            tof_b_entry: "0.0".to_string(),
            tof_arm_time: None,
            tof_armed: false,
            tof_t0_ms: None,
            tof_t1_ms: None,
            tof_tof_ms: 0.0,
            tof_wavespeed: 0.0,
            tof_wave_detected: false,
            tof_baseline: 0.0,
            tof_baseline_shooter: 0.0,
            tof_baseline_samples: VecDeque::new(),
            tof_baseline_shooter_samples: VecDeque::new(),
            tof_history: Vec::new(),
            tof_times: VecDeque::with_capacity(TOF_BUFFER_SIZE),
            tof_m_angle: VecDeque::with_capacity(TOF_BUFFER_SIZE),
            tof_s_angle: VecDeque::with_capacity(TOF_BUFFER_SIZE),
            tof_t0_line: None,
            tof_t1_line: None,
            tof_t1_sent: None,

            tof_error_msg: None,

            tof_shooter: TofShooter::Master,
            tof_pulse_us: 2000,
            tof_firing_time: None,

            diag_data: None,
        }
    }
}

// ============================================================================
// LÓGICA SERIAL
// ============================================================================
impl SysIdApp {
    fn poll_serial(&mut self) {
        let events: Vec<SerialEvent> = if let Some(ref serial) = self.serial {
            serial.receiver.try_iter().collect()
        } else {
            return;
        };

        for event in events {
            match event {
                SerialEvent::Packet(pkt) => {
                    self.pkt_count += 1;
                    self.last_rx_time = Instant::now();
                    self.connection_ok = true;

                    // Tiempo relativo en segundos
                    if self.live_t0_ms.is_none() {
                        self.live_t0_ms = Some(pkt.timestamp_ms);
                    }
                    let t_sec = (pkt.timestamp_ms.wrapping_sub(self.live_t0_ms.unwrap())) as f64
                        / 1000.0;

                    // Alimentar buffers circulares
                    push_buf(&mut self.live_times, t_sec);
                    push_buf(&mut self.live_m_pwm, pkt.m_pwm as f64);
                    push_buf(&mut self.live_s_pwm, pkt.s_pwm as f64);
                    push_buf(&mut self.live_m_esc, pkt.m_esc as f64);
                    push_buf(&mut self.live_s_esc, pkt.s_esc as f64);
                    push_buf(&mut self.live_m_rpm, pkt.m_rpm as f64);
                    push_buf(&mut self.live_s_rpm, pkt.s_rpm as f64);
                    push_buf(&mut self.live_m_hz, pkt.m_hz);
                    push_buf(&mut self.live_s_hz, pkt.s_hz);

                    // --- ToF Calibration buffers ---
                    let tof_t = t_sec;
                    push_buf_sized(&mut self.tof_times, tof_t, TOF_BUFFER_SIZE);
                    push_buf_sized(
                        &mut self.tof_m_angle,
                        pkt.m_angle as f64,
                        TOF_BUFFER_SIZE,
                    );
                    push_buf_sized(
                        &mut self.tof_s_angle,
                        pkt.s_angle as f64,
                        TOF_BUFFER_SIZE,
                    );

                    // Feed baseline samples during Arming state (before firer ESC spins)
                    // This ensures a clean baseline uncontaminated by firer vibration
                    if self.tof_state == TofState::Arming {
                        let baseline_angle = match self.tof_shooter {
                            TofShooter::Master => pkt.s_angle, // Master dispara → baseline del sensor Slave
                            TofShooter::Slave => pkt.m_angle,  // Slave dispara → baseline del sensor Master
                        };
                        push_buf_sized(&mut self.tof_baseline_samples, baseline_angle as f64, 100);
                        let shooter_angle = match self.tof_shooter {
                            TofShooter::Master => pkt.m_angle, // Master fires → shooter is Master encoder
                            TofShooter::Slave => pkt.s_angle,  // Slave fires → shooter is Slave encoder
                        };
                        push_buf_sized(&mut self.tof_baseline_shooter_samples, shooter_angle as f64, 100);
                    }

                    // Check for wave detection during Firing (short ropes) and Listening
                    if (self.tof_state == TofState::Firing || self.tof_state == TofState::Listening) && !self.tof_wave_detected {
                        // Minimum time guard: skip detection for TOF_MIN_DETECT_MS after firing
                        let past_min_time = self.tof_firing_time.map_or(true, |ft| {
                            ft.elapsed() >= Duration::from_millis(TOF_MIN_DETECT_MS)
                        });

                        if past_min_time {
                            // 1. Detect shooter movement for true hardware T0
                            // 1. Detectar el flanco de subida del disparador para el verdadero T0 físico
                            if self.tof_t0_ms.is_none() {
                                let shooter_angle = match self.tof_shooter {
                                    TofShooter::Master => pkt.m_angle,
                                    TofShooter::Slave => pkt.s_angle,
                                };
                                
                                // CORRECCIÓN: Sin .abs() para asegurar que solo reacciona al empuje positivo
                                let shooter_delta = shooter_angle as f64 - self.tof_baseline_shooter;
                                
                                if shooter_delta >= self.tof_threshold {
                                    self.tof_t0_ms = Some(pkt.timestamp_ms);
                                    self.tof_t0_line = Some(tof_t);
                                }
                            }

                            // 2. Detect receiver impact for T1 (only after T0 is known)
                            // 2. Detectar el impacto en el receptor para T1 (Solo después de T0)
                            if let Some(t0_ms) = self.tof_t0_ms {
                                // CORRECCIÓN: Filtro de > 0ms para evitar que ruido eléctrico dispare ambos en el mismo paquete
                                if pkt.timestamp_ms > t0_ms {
                                    let receiver_angle = match self.tof_shooter {
                                        TofShooter::Master => pkt.s_angle,
                                        TofShooter::Slave => pkt.m_angle,
                                    };
                                    
                                    // CORRECCIÓN: Sin .abs() para ignorar el flanco de bajada / rebote mecánico
                                    let delta = receiver_angle as f64 - self.tof_baseline;
                                    
                                    if delta >= self.tof_threshold {
                                        self.tof_wave_detected = true;
                                        self.tof_t1_ms = Some(pkt.timestamp_ms);
                                        
                                        self.tof_tof_ms = (pkt.timestamp_ms as i64 - t0_ms as i64) as f64;
                                        self.tof_wavespeed = self.tof_rope_length / (self.tof_tof_ms / 1000.0);
                                        self.tof_t1_line = Some(tof_t);
                                        self.tof_state = TofState::Done;

                                        let ts = chrono::Local::now().format("%H:%M:%S").to_string();
                                        self.tof_history.push(TofResult {
                                            timestamp: ts,
                                            target_hz: self.tof_target_hz,
                                            tof_ms: self.tof_tof_ms,
                                            wavespeed: self.tof_wavespeed,
                                        });
                                    }
                                }
                            }
                        }
                    }

                    // CSV
                    if self.is_recording {
                        self.data_log.push(pkt.clone());
                    }

                    self.last_packet = Some(pkt);
                }
                SerialEvent::ParseError => {
                    self.pkt_errors += 1;
                }
                SerialEvent::Diagnostic(diag) => {
                    self.diag_data = Some(diag);
                }
                SerialEvent::Disconnected => {
                    self.disconnect();
                    return;
                }
            }
        }
    }

    fn check_watchdog(&mut self) {
        if self.serial.is_none() {
            return;
        }
        let elapsed = self.last_rx_time.elapsed().as_secs_f64();
        if elapsed > 2.0 && self.connection_ok {
            self.connection_ok = false;
            self.warning_text =
                "⚠ SIN DATOS DEL ATmega\n(REVISA SPI/USB/BATERÍA)".to_string();
        } else if elapsed <= 2.0 {
            self.warning_text.clear();
        }
    }

    fn connect(&mut self) {
        if self.available_ports.is_empty() {
            self.message = Some((
                "Error".to_string(),
                "Selecciona un puerto Serial.".to_string(),
            ));
            return;
        }
        let port_name = self.available_ports[self.selected_port_idx].clone();
        match SerialConnection::open(&port_name) {
            Ok(conn) => {
                self.serial = Some(conn);
                self.last_rx_time = Instant::now();
                self.connection_ok = true;
                self.pkt_count = 0;
                self.pkt_errors = 0;
                self.live_t0_ms = None;
                self.status_text = format!("CONECTADO: {}", port_name);
                self.status_color = C_GREEN;
                self.warning_text.clear();
            }
            Err(e) => {
                self.message = Some((
                    "Error Serial".to_string(),
                    format!("No se pudo abrir {}:\n{}", port_name, e),
                ));
            }
        }
    }

    fn disconnect(&mut self) {
        if self.is_recording {
            self.stop_recording();
        }
        if let Some(ref mut serial) = self.serial {
            serial.close();
        }
        self.serial = None;
        self.connection_ok = false;
        self.dc_active = false;
        self.ac_active = false;
        self.status_text = "DESCONECTADO".to_string();
        self.status_color = C_RED;
        self.tof_state = TofState::Idle;
    }

    fn refresh_ports(&mut self) {
        self.available_ports = list_ports();
        if !self.available_ports.is_empty() {
            self.selected_port_idx = self
                .available_ports
                .iter()
                .position(|p| p.contains("ttyUSB"))
                .unwrap_or(0);
        }
    }
}

// ============================================================================
// COMANDOS
// ============================================================================
impl SysIdApp {
    fn send_dc(&mut self) {
        self.sync_dc_from_entry();
        let val = self.dc_value as i16;
        if let Some(ref mut serial) = self.serial {
            let _ = serial.send_command(CMD_DC_BOTH, val);
        }
        self.dc_active = val != 0;
        self.status_text = format!("DC ACTIVO: PWM={}", val);
        self.status_color = C_BLUE;
    }

    fn stop_dc(&mut self) {
        if let Some(ref mut serial) = self.serial {
            let _ = serial.send_command(CMD_DC_BOTH, 0);
        }
        self.dc_active = false;
        self.dc_value = 0;
        self.dc_entry = "0".to_string();
        self.status_text = "DC DETENIDO".to_string();
        self.status_color = C_BLUE;
    }

    fn send_ac(&mut self) {
        self.sync_ac_from_entry();
        let val = self.ac_value as i16;
        let cmd = self.ac_target.cmd();
        if let Some(ref mut serial) = self.serial {
            let _ = serial.send_command(cmd, val);
        }
        self.ac_active = val != 1500;
        self.status_text = format!(
            "AC ACTIVO ({}): {}µs",
            self.ac_target.label(),
            self.ac_value
        );
        self.status_color = C_ORANGE;
    }

    fn stop_ac(&mut self) {
        let cmd = self.ac_target.cmd();
        if let Some(ref mut serial) = self.serial {
            let _ = serial.send_command(cmd, 0);
        }
        self.ac_active = false;
        self.ac_value = 1500;
        self.ac_entry = "1500".to_string();
        self.status_text = "AC DETENIDO".to_string();
        self.status_color = C_ORANGE;
    }

    fn stop_all(&mut self) {
        if let Some(ref mut serial) = self.serial {
            let _ = serial.send_command(CMD_STOP_ALL, 0);
        }
        self.dc_active = false;
        self.ac_active = false;
        self.dc_value = 0;
        self.dc_entry = "0".to_string();
        self.ac_value = 1500;
        self.ac_entry = "1500".to_string();
        self.status_text = "TODO DETENIDO".to_string();
        self.status_color = C_GREEN;
        self.tof_state = TofState::Idle;
        if self.is_recording {
            self.stop_recording();
        }
        self.close_after = Some(Instant::now() + Duration::from_millis(500));
    }

    fn request_encoder_diag(&mut self) {
        if let Some(ref mut serial) = self.serial {
            let _ = serial.send_command(CMD_ASK_ENCODER, 0);
        }
    }

    fn sync_dc_from_entry(&mut self) {
        if let Ok(v) = self.dc_entry.parse::<i32>() {
            self.dc_value = v.clamp(-255, 255);
            self.dc_entry = self.dc_value.to_string();
        }
    }

    fn sync_ac_from_entry(&mut self) {
        if let Ok(v) = self.ac_entry.parse::<i32>() {
            self.ac_value = v.clamp(1000, 2000);
            self.ac_entry = self.ac_value.to_string();
        }
    }
}

// ============================================================================
// ToF CALIBRATION — STATE MACHINE
// ============================================================================
impl SysIdApp {
    fn tof_update_state(&mut self) {
        if self.serial.is_none() {
            return;
        }

        let esc_spin_cmd = match self.tof_shooter {
            TofShooter::Master => CMD_TOF_START,
            TofShooter::Slave => CMD_TOF_FIRE_SLAVE,
        };

        match self.tof_state {
            TofState::Idle => {}
            TofState::Arming => {
                if let Some(arm_time) = self.tof_arm_time {
                    if arm_time.elapsed() >= Duration::from_millis(TOF_ARM_DURATION_MS) {
                        let hz_val = self.tof_target_hz as i16;
                        if let Some(ref mut serial) = self.serial {
                            let _ = serial.send_command(esc_spin_cmd, hz_val);
                        }
                        self.tof_state = TofState::Silence;
                        self.tof_arm_time = Some(Instant::now());
                    }
                }
            }
            TofState::Silence => {
                if let Some(arm_time) = self.tof_arm_time {
                    if arm_time.elapsed() >= Duration::from_millis(TOF_SILENCE_DURATION_MS) {
                        let n = self.tof_baseline_samples.len();
                        if n >= 5 {
                            let sum: f64 = self.tof_baseline_samples.iter().rev().take(5).sum();
                            self.tof_baseline = sum / 5.0;
                        } else if n > 0 {
                            let sum: f64 = self.tof_baseline_samples.iter().sum();
                            self.tof_baseline = sum / n as f64;
                        } else {
                            self.tof_baseline = 0.0;
                        }

                        let n_shooter = self.tof_baseline_shooter_samples.len();
                        if n_shooter >= MIN_BASELINE_SAMPLES {
                            let sum: f64 = self.tof_baseline_shooter_samples.iter().rev().take(5).sum();
                            self.tof_baseline_shooter = sum / 5.0;
                        } else if n_shooter > 0 {
                            let sum: f64 = self.tof_baseline_shooter_samples.iter().sum();
                            self.tof_baseline_shooter = sum / n_shooter as f64;
                        } else {
                            self.tof_baseline_shooter = self.tof_baseline_shooter_samples.front().copied().unwrap_or(0.0);
                        }

                        if let Some(ref mut serial) = self.serial {
                            let _ = serial.send_command(esc_spin_cmd, self.tof_pulse_us);
                        }
                        self.tof_state = TofState::Firing;
                        self.tof_arm_time = Some(Instant::now());
                        self.tof_firing_time = Some(Instant::now());
                    }
                }
            }
            TofState::Firing => {
                if let Some(arm_time) = self.tof_arm_time {
                    if arm_time.elapsed() >= Duration::from_millis(TOF_PULSE_DURATION_MS) {
                        let detach_firer: i16 = match self.tof_shooter {
                            TofShooter::Master => 0,
                            TofShooter::Slave => 1,
                        };
                        if let Some(ref mut serial) = self.serial {
                            let _ = serial.send_command(CMD_TOF_DETACH, detach_firer);
                        }
                        self.tof_t1_sent = Some(Instant::now());
                        self.tof_state = TofState::Listening;
                    }
                }
            }
            TofState::Listening => {
                if let Some(t1_sent) = self.tof_t1_sent {
                    if t1_sent.elapsed() >= Duration::from_millis(TOF_LISTEN_TIMEOUT_MS) {
                        self.tof_tof_ms = 0.0;
                        self.tof_wavespeed = 0.0;
                        if self.tof_t0_ms.is_none() {
                            self.tof_error_msg = Some(
                                "Error: Disparador no detectado. Verificar encoder del motor.".to_string()
                            );
                        } else {
                            self.tof_error_msg = Some(
                                "Error: Timeout. Cuerda no detectada o señal muy débil.".to_string()
                            );
                        }
                        self.tof_state = TofState::Done;
                    }
                }
            }
            TofState::Done => {}
        }
    }

    fn tof_arm(&mut self) {
        if self.tof_state != TofState::Idle || self.serial.is_none() {
            return;
        }
        if let Some(ref mut serial) = self.serial {
            match self.tof_shooter {
                TofShooter::Master => { let _ = serial.send_command(CMD_TOF_ARM, 0); }
                TofShooter::Slave => { let _ = serial.send_command(CMD_TOF_FIRE_SLAVE, TOF_NEUTRAL_ESC); }
            }
        }
        self.tof_arm_time = Some(Instant::now());
        self.tof_armed = true;
        self.tof_wave_detected = false;
        self.tof_t0_ms = None;
        self.tof_t1_ms = None;
        self.tof_tof_ms = 0.0;
        self.tof_wavespeed = 0.0;
        self.tof_t0_line = None;
        self.tof_t1_line = None;
        self.tof_baseline_samples.clear();
        self.tof_baseline_shooter_samples.clear();
        self.tof_baseline_shooter = 0.0;
        self.tof_error_msg = None;
        self.tof_state = TofState::Arming;
    }

    fn tof_stop(&mut self) {
        if let Some(ref mut serial) = self.serial {
            let _ = serial.send_command(CMD_TOF_STOP, 0);
            let reattach_firer: i16 = match self.tof_shooter {
                TofShooter::Master => 0,
                TofShooter::Slave => 1,
            };
            let _ = serial.send_command(CMD_TOF_ATTACH, reattach_firer);
        }
        self.tof_state = TofState::Idle;
        self.tof_armed = false;
        self.tof_arm_time = None;
        self.tof_t1_sent = None;
        self.tof_firing_time = None;
    }

    fn tof_reset(&mut self) {
        self.tof_state = TofState::Idle;
        self.tof_armed = false;
        self.tof_arm_time = None;
        self.tof_t1_sent = None;
        self.tof_firing_time = None;
        self.tof_wave_detected = false;
        self.tof_t0_ms = None;
        self.tof_t1_ms = None;
        self.tof_tof_ms = 0.0;
        self.tof_wavespeed = 0.0;
        self.tof_t0_line = None;
        self.tof_t1_line = None;
        self.tof_error_msg = None;
        self.tof_baseline_samples.clear();
        self.tof_baseline_shooter_samples.clear();
        self.tof_baseline_shooter = 0.0;
    }

    fn tof_export_csv(&self) {
        let ts = chrono::Local::now().format("%Y%m%d_%H%M%S");
        let filename = format!("tof_results_{}.csv", ts);
        if let Ok(mut writer) = csv::Writer::from_path(&filename) {
            let _ = writer.write_record(["timestamp", "target_hz", "tof_ms", "wavespeed"]);
            for r in &self.tof_history {
                let _ = writer.write_record([
                    &r.timestamp,
                    &format!("{:.1}", r.target_hz),
                    &format!("{:.2}", r.tof_ms),
                    &format!("{:.4}", r.wavespeed),
                ]);
            }
            let _ = writer.flush();
        }
    }
}

// ============================================================================
// GRABACIÓN CSV
// ============================================================================
impl SysIdApp {
    fn start_recording(&mut self) {
        self.is_recording = true;
        self.data_log.clear();
        self.status_text = "GRABANDO...".to_string();
        self.status_color = C_ORANGE;
    }

    fn stop_recording(&mut self) {
        self.is_recording = false;

        if !self.data_log.is_empty() {
            match self.save_csv() {
                Ok((filename, count)) => {
                    self.message = Some((
                        "Éxito".to_string(),
                        format!(
                            "Ensayo guardado en:\n{}\n{} muestras.\n\n\
                             Usa timestamp_ms como eje X en MATLAB/Excel\n\
                             para calcular τ y Tiempo Muerto.",
                            filename, count
                        ),
                    ));
                }
                Err(e) => {
                    self.message = Some(("Error CSV".to_string(), e.to_string()));
                }
            }
        }
    }

    fn save_csv(&self) -> Result<(String, usize), Box<dyn std::error::Error>> {
        let ts = chrono::Local::now().format("%Y%m%d_%H%M%S");
        let filename = format!("step_response_{}.csv", ts);
        let mut writer = csv::Writer::from_path(&filename)?;

        writer.write_record([
            "timestamp_ms",
            "mPWM",
            "sPWM",
            "mESC",
            "sESC",
            "mRPM",
            "sRPM",
            "mHz",
            "sHz",
        ])?;

        for pkt in &self.data_log {
            writer.write_record(&[
                pkt.timestamp_ms.to_string(),
                pkt.m_pwm.to_string(),
                pkt.s_pwm.to_string(),
                pkt.m_esc.to_string(),
                pkt.s_esc.to_string(),
                pkt.m_rpm.to_string(),
                pkt.s_rpm.to_string(),
                format!("{:.2}", pkt.m_hz),
                format!("{:.2}", pkt.s_hz),
            ])?;
        }

        writer.flush()?;
        let count = self.data_log.len();
        Ok((filename, count))
    }
}

// ============================================================================
// LOGO
// ============================================================================
impl SysIdApp {
    fn load_logo(&mut self, ctx: &egui::Context) {
        self.logo_loaded = true;

        let search_paths = [
            std::path::PathBuf::from("logo.png"),
            std::path::PathBuf::from("../test/logo.png"),
            std::path::PathBuf::from("test/logo.png"),
        ];

        // También buscar junto al ejecutable
        let exe_paths: Vec<std::path::PathBuf> = std::env::current_exe()
            .ok()
            .and_then(|p| p.parent().map(|d| d.to_path_buf()))
            .map(|dir| vec![dir.join("logo.png")])
            .unwrap_or_default();

        for path in search_paths.iter().chain(exe_paths.iter()) {
            if path.exists() {
                if let Ok(img) = image::open(path) {
                    let rgba = img.to_rgba8();
                    let size = [rgba.width() as usize, rgba.height() as usize];
                    let color_image =
                        egui::ColorImage::from_rgba_unmultiplied(size, rgba.as_raw());
                    self.logo_texture = Some(ctx.load_texture(
                        "logo",
                        color_image,
                        egui::TextureOptions::LINEAR,
                    ));
                    return;
                }
            }
        }
    }
}

// ============================================================================
// RENDERIZADO UI
// ============================================================================
impl SysIdApp {
    fn render_header(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.add_space(15.0);

            // Logo
            if let Some(ref tex) = self.logo_texture {
                let size = tex.size_vec2();
                let ratio = 50.0 / size.y;
                let display_size = egui::vec2(size.x * ratio, 50.0);
                ui.image(egui::load::SizedTexture::new(tex.id(), display_size));
            } else {
                ui.label(
                    RichText::new("ATACAMA DYNAMICS")
                        .size(16.0)
                        .strong()
                        .color(C_BLUE),
                );
            }

            ui.add_space(20.0);

            ui.vertical(|ui| {
                ui.add_space(8.0);
                ui.label(
                    RichText::new("SISTEMA DAQ & IDENTIFICACIÓN")
                        .size(18.0)
                        .strong()
                        .color(FG_TEXT),
                );
                ui.label(
                    RichText::new("Banco de Pruebas de Lazo Abierto v4.0 (Rust)")
                        .size(10.0)
                        .color(FG_DIM),
                );
            });
        });
    }

    fn render_tab_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            for tab in [Tab::Control, Tab::TofCalibration] {
                let is_active = self.active_tab == tab;
                let text = RichText::new(tab.label())
                    .size(13.0)
                    .strong()
                    .color(if is_active { Color32::WHITE } else { FG_DIM });
                let btn = egui::Button::new(text)
                    .fill(if is_active { C_BLUE } else { BG_PANEL })
                    .rounding(Rounding::same(4.0))
                    .min_size(egui::vec2(120.0, 28.0));
                if ui.add(btn).clicked() {
                    self.active_tab = tab;
                }
            }
        });
    }

    fn render_controls(&mut self, ui: &mut egui::Ui) {
        let connected = self.serial.is_some();

        // ---- CONEXIÓN SERIAL ----
        section(ui, "CONEXIÓN SERIAL", C_BLUE, |ui| {
            ui.horizontal(|ui| {
                let selected = if self.available_ports.is_empty() {
                    "Sin puertos".to_string()
                } else {
                    self.available_ports
                        .get(self.selected_port_idx)
                        .cloned()
                        .unwrap_or_default()
                };

                egui::ComboBox::from_id_salt("port_combo")
                    .selected_text(&selected)
                    .width(180.0)
                    .show_ui(ui, |ui| {
                        for (i, port) in self.available_ports.iter().enumerate() {
                            ui.selectable_value(&mut self.selected_port_idx, i, port);
                        }
                    });

                if ui
                    .button(RichText::new("⟳").size(14.0).color(FG_DIM))
                    .clicked()
                {
                    self.refresh_ports();
                }
            });

            ui.add_space(8.0);

            if connected {
                if colored_button(ui, "DESCONECTAR", C_RED) {
                    self.disconnect();
                }
            } else if colored_button(ui, "CONECTAR", C_BLUE) {
                self.connect();
            }
        });

        ui.add_space(8.0);

        // ---- CONTROL DC ----
        section(ui, "MOTOR DC (PWM -255 a 255)", C_BLUE, |ui| {
            ui.horizontal(|ui| {
                let entry_response = ui.add(
                    egui::TextEdit::singleline(&mut self.dc_entry)
                        .desired_width(80.0)
                        .font(egui::FontId::proportional(16.0))
                        .horizontal_align(egui::Align::Center),
                );
                if entry_response.lost_focus()
                    && ui.input(|i| i.key_pressed(egui::Key::Enter))
                {
                    self.sync_dc_from_entry();
                }
                ui.label(RichText::new("(-255 a 255)").size(9.0).color(FG_DIM));
            });

            ui.add_space(4.0);

            let slider = ui.add(
                egui::Slider::new(&mut self.dc_value, -255..=255)
                    .show_value(false)
                    .trailing_fill(true),
            );
            if slider.changed() {
                self.dc_entry = self.dc_value.to_string();
            }

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(connected, colored_button_widget("▶ ENVIAR DC", C_BLUE))
                    .clicked()
                {
                    self.send_dc();
                }
                if ui
                    .add_enabled(connected, colored_button_widget("⏹ STOP DC", C_RED))
                    .clicked()
                {
                    self.stop_dc();
                }
            });
        });

        ui.add_space(8.0);

        // ---- CONTROL AC (ESC) ----
        section(ui, "MOTOR AC / ESC (µs 1000-2000)", C_ORANGE, |ui| {
            ui.horizontal(|ui| {
                for target in [AcTarget::Both, AcTarget::Master, AcTarget::Slave] {
                    ui.radio_value(&mut self.ac_target, target, target.label());
                }
            });

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                let entry_response = ui.add(
                    egui::TextEdit::singleline(&mut self.ac_entry)
                        .desired_width(80.0)
                        .font(egui::FontId::proportional(16.0))
                        .horizontal_align(egui::Align::Center),
                );
                if entry_response.lost_focus()
                    && ui.input(|i| i.key_pressed(egui::Key::Enter))
                {
                    self.sync_ac_from_entry();
                }
                ui.label(RichText::new("(1000-2000 µs)").size(9.0).color(FG_DIM));
            });

            ui.add_space(4.0);

            let slider = ui.add(
                egui::Slider::new(&mut self.ac_value, 1000..=2000)
                    .show_value(false)
                    .trailing_fill(true),
            );
            if slider.changed() {
                self.ac_entry = self.ac_value.to_string();
            }

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(connected, colored_button_widget("▶ ENVIAR AC", C_ORANGE))
                    .clicked()
                {
                    self.send_ac();
                }
                if ui
                    .add_enabled(connected, colored_button_widget("⏹ STOP AC", C_RED))
                    .clicked()
                {
                    self.stop_ac();
                }
            });
        });

        ui.add_space(8.0);

        // ---- GRABACIÓN ----
        section(ui, "GRABACIÓN", C_GREEN, |ui| {
            if self.is_recording {
                if ui
                    .add_enabled(connected, colored_button_widget("⏹ DETENER GRABACIÓN", C_RED))
                    .clicked()
                {
                    self.stop_recording();
                }
            } else if ui
                .add_enabled(connected, colored_button_widget("⏺ GRABAR CSV", C_GREEN))
                .clicked()
            {
                self.start_recording();
            }

            ui.add_space(6.0);
            if ui
                .add_enabled(connected, colored_button_widget("⏹ STOP TODO", C_RED))
                .clicked()
            {
                self.stop_all();
            }
        });

        ui.add_space(8.0);

        // ---- TELEMETRÍA EN VIVO ----
        section(ui, "TELEMETRÍA EN VIVO", C_CYAN, |ui| {
            let mono = egui::FontId::monospace(10.0);
            if let Some(ref pkt) = self.last_packet {
                ui.label(RichText::new(format!("t(ms): {}", pkt.timestamp_ms)).font(mono.clone()).color(FG_TEXT));
                ui.label(RichText::new(format!("M PWM: {}", pkt.m_pwm)).font(mono.clone()).color(FG_TEXT));
                ui.label(RichText::new(format!("S PWM: {}", pkt.s_pwm)).font(mono.clone()).color(FG_TEXT));
                ui.label(RichText::new(format!("M ESC: {} µs", pkt.m_esc)).font(mono.clone()).color(FG_TEXT));
                ui.label(RichText::new(format!("S ESC: {} µs", pkt.s_esc)).font(mono.clone()).color(FG_TEXT));
                ui.label(RichText::new(format!("M RPM: {}", pkt.m_rpm)).font(mono.clone()).color(FG_TEXT));
                ui.label(RichText::new(format!("S RPM: {}", pkt.s_rpm)).font(mono.clone()).color(FG_TEXT));
                ui.label(RichText::new(format!("M Hz:  {:.2}", pkt.m_hz)).font(mono.clone()).color(FG_TEXT));
                ui.label(RichText::new(format!("S Hz:  {:.2}", pkt.s_hz)).font(mono).color(FG_TEXT));
            } else {
                for lbl in [
                    "t(ms): ---",
                    "M PWM: ---",
                    "S PWM: ---",
                    "M ESC: ---",
                    "S ESC: ---",
                    "M RPM: ---",
                    "S RPM: ---",
                    "M Hz:  ---",
                    "S Hz:  ---",
                ] {
                    ui.label(RichText::new(lbl).font(egui::FontId::monospace(10.0)).color(FG_DIM));
                }
            }
        });

        ui.add_space(8.0);

        // ---- ESTADÍSTICAS ----
        section(ui, "ESTADÍSTICAS", C_BLUE, |ui| {
            let mono = egui::FontId::monospace(10.0);
            ui.label(
                RichText::new(format!("Paquetes: {}", self.pkt_count))
                    .font(mono.clone())
                    .color(FG_TEXT),
            );
            ui.label(
                RichText::new(format!("Errores:  {}", self.pkt_errors))
                    .font(mono.clone())
                    .color(FG_TEXT),
            );

            let rate_text = if self.live_times.len() >= 10 {
                let dt = self.live_times.back().unwrap() - self.live_times[self.live_times.len() - 10];
                if dt > 0.0 {
                    format!("Rate:     {:.0} Hz", 9.0 / dt)
                } else {
                    "Rate:     --- Hz".to_string()
                }
            } else {
                "Rate:     --- Hz".to_string()
            };
            ui.label(
                RichText::new(rate_text)
                    .font(mono)
                    .strong()
                    .color(C_BLUE),
            );
        });

        // ---- WARNING ----
        if !self.warning_text.is_empty() {
            ui.add_space(10.0);
            ui.label(
                RichText::new(&self.warning_text)
                    .size(12.0)
                    .strong()
                    .color(C_RED),
            );
        }

        ui.add_space(8.0);

        section(ui, "AS5600 DIAGNOSTIC", C_YELLOW, |ui| {
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(connected, colored_button_widget("🔍 DIAGNOSTICAR AS5600", C_YELLOW))
                    .clicked()
                {
                    self.request_encoder_diag();
                }
            });

            if let Some(ref d) = self.diag_data {
                ui.add_space(4.0);
                let mono = egui::FontId::monospace(10.0);
                ui.label(RichText::new("── Master ──").size(11.0).strong().color(FG_TEXT));
                ui.label(RichText::new(format!("  MPOS: {}  ZPOS: {}  MAG: {}", d.mpos_m, d.zpos_m, d.mag_m)).font(mono.clone()).color(FG_TEXT));
                ui.label(RichText::new(format!("  AGC: {}   RAW: {}", d.agc_m, d.raw_m)).font(mono.clone()).color(FG_TEXT));
                let m_conn_color = if d.conn_m == 1 { C_GREEN } else { C_RED };
                let m_conn_txt = if d.conn_m == 1 { "YES" } else { "NO" };
                ui.label(RichText::new(format!("  Connected: {}", m_conn_txt)).font(mono.clone()).color(m_conn_color));

                ui.add_space(2.0);
                ui.label(RichText::new("── Slave ──").size(11.0).strong().color(FG_TEXT));
                ui.label(RichText::new(format!("  MPOS: {}  ZPOS: {}  MAG: {}", d.mpos_s, d.zpos_s, d.mag_s)).font(mono.clone()).color(FG_TEXT));
                ui.label(RichText::new(format!("  AGC: {}   RAW: {}", d.agc_s, d.raw_s)).font(mono.clone()).color(FG_TEXT));
                let s_conn_color = if d.conn_s == 1 { C_GREEN } else { C_RED };
                let s_conn_txt = if d.conn_s == 1 { "YES" } else { "NO" };
                ui.label(RichText::new(format!("  Connected: {}", s_conn_txt)).font(mono).color(s_conn_color));
            } else {
                ui.label(RichText::new("Sin datos. Presiona DIAGNOSTICAR.").size(10.0).color(FG_DIM));
            }
        });

        // ---- STATUS ----
        ui.add_space(10.0);
        ui.with_layout(egui::Layout::bottom_up(egui::Align::Center), |ui| {
            ui.label(
                RichText::new(&self.status_text)
                    .size(11.0)
                    .strong()
                    .color(self.status_color),
            );
        });
    }

    fn render_plots(&self, ui: &mut egui::Ui) {
        let available = ui.available_size();
        let plot_h = (available.y - 20.0) / 2.0;

        // Datos compartidos
        let times: Vec<f64> = self.live_times.iter().copied().collect();

        // Calcular ventana temporal deslizante de 30s
        let (t_min, t_max) = if let Some(&last) = times.last() {
            let tmin = f64::max(0.0, last - 30.0);
            (tmin, f64::max(last, tmin + 5.0))
        } else {
            (0.0, 5.0)
        };

        // ---- Fila superior: DC + AC ----
        ui.horizontal(|ui| {
            let w = (ui.available_width() - 10.0) / 2.0;

            // Plot DC
            ui.allocate_ui(egui::vec2(w, plot_h), |ui| {
                ui.label(
                    RichText::new("Actuación DC (PWM)")
                        .strong()
                        .size(11.0)
                        .color(FG_TEXT),
                );
                let m_pwm: Vec<[f64; 2]> = times
                    .iter()
                    .zip(self.live_m_pwm.iter())
                    .map(|(&t, &v)| [t, v])
                    .collect();
                let s_pwm: Vec<[f64; 2]> = times
                    .iter()
                    .zip(self.live_s_pwm.iter())
                    .map(|(&t, &v)| [t, v])
                    .collect();

                let (y_min, y_max) = dc_bounds(&self.live_m_pwm, &self.live_s_pwm);

                Plot::new("dc_plot")
                    .legend(Legend::default())
                    .x_axis_label("Tiempo (s)")
                    .y_axis_label("PWM")
                    .allow_zoom(false)
                    .allow_scroll(false)
                    .allow_drag(false)
                    .allow_boxed_zoom(false)
                    .show(ui, |plot_ui| {
                        if !m_pwm.is_empty() {
                            plot_ui.line(
                                Line::new(PlotPoints::new(m_pwm))
                                    .color(C_BLUE)
                                    .name("M PWM")
                                    .width(1.5f32),
                            );
                            plot_ui.line(
                                Line::new(PlotPoints::new(s_pwm))
                                    .color(C_GREEN)
                                    .name("S PWM")
                                    .width(1.5f32),
                            );
                        }
                        plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                            [t_min, y_min],
                            [t_max, y_max],
                        ));
                    });
            });

            // Plot AC
            ui.allocate_ui(egui::vec2(w, plot_h), |ui| {
                ui.label(
                    RichText::new("Actuación AC (ESC µs)")
                        .strong()
                        .size(11.0)
                        .color(FG_TEXT),
                );
                let m_esc: Vec<[f64; 2]> = times
                    .iter()
                    .zip(self.live_m_esc.iter())
                    .map(|(&t, &v)| [t, v])
                    .collect();
                let s_esc: Vec<[f64; 2]> = times
                    .iter()
                    .zip(self.live_s_esc.iter())
                    .map(|(&t, &v)| [t, v])
                    .collect();

                let (y_min, y_max) = ac_bounds(&self.live_m_esc, &self.live_s_esc);

                Plot::new("ac_plot")
                    .legend(Legend::default())
                    .x_axis_label("Tiempo (s)")
                    .y_axis_label("µs")
                    .allow_zoom(false)
                    .allow_scroll(false)
                    .allow_drag(false)
                    .allow_boxed_zoom(false)
                    .show(ui, |plot_ui| {
                        if !m_esc.is_empty() {
                            plot_ui.line(
                                Line::new(PlotPoints::new(m_esc))
                                    .color(C_ORANGE)
                                    .name("M ESC")
                                    .width(1.5f32),
                            );
                            plot_ui.line(
                                Line::new(PlotPoints::new(s_esc))
                                    .color(C_YELLOW)
                                    .name("S ESC")
                                    .width(1.5f32),
                            );
                        }
                        plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                            [t_min, y_min],
                            [t_max, y_max],
                        ));
                    });
            });
        });

        // ---- Fila inferior: RPM + Hz ----
        ui.horizontal(|ui| {
            let w = (ui.available_width() - 10.0) / 2.0;

            // Plot RPM
            ui.allocate_ui(egui::vec2(w, plot_h), |ui| {
                ui.label(
                    RichText::new("Medición RPM")
                        .strong()
                        .size(11.0)
                        .color(FG_TEXT),
                );
                let m_rpm: Vec<[f64; 2]> = times
                    .iter()
                    .zip(self.live_m_rpm.iter())
                    .map(|(&t, &v)| [t, v])
                    .collect();
                let s_rpm: Vec<[f64; 2]> = times
                    .iter()
                    .zip(self.live_s_rpm.iter())
                    .map(|(&t, &v)| [t, v])
                    .collect();

                let y_max = rpm_max(&self.live_m_rpm, &self.live_s_rpm);

                Plot::new("rpm_plot")
                    .legend(Legend::default())
                    .x_axis_label("Tiempo (s)")
                    .y_axis_label("RPM")
                    .allow_zoom(false)
                    .allow_scroll(false)
                    .allow_drag(false)
                    .allow_boxed_zoom(false)
                    .show(ui, |plot_ui| {
                        if !m_rpm.is_empty() {
                            plot_ui.line(
                                Line::new(PlotPoints::new(m_rpm))
                                    .color(C_BLUE)
                                    .name("M RPM")
                                    .width(1.5f32),
                            );
                            plot_ui.line(
                                Line::new(PlotPoints::new(s_rpm))
                                    .color(C_GREEN)
                                    .name("S RPM")
                                    .width(1.5f32),
                            );
                        }
                        plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                            [t_min, 0.0],
                            [t_max, y_max],
                        ));
                    });
            });

            // Plot Hz
            ui.allocate_ui(egui::vec2(w, plot_h), |ui| {
                ui.label(
                    RichText::new("Medición Hz (BLDC)")
                        .strong()
                        .size(11.0)
                        .color(FG_TEXT),
                );
                let m_hz: Vec<[f64; 2]> = times
                    .iter()
                    .zip(self.live_m_hz.iter())
                    .map(|(&t, &v)| [t, v])
                    .collect();
                let s_hz: Vec<[f64; 2]> = times
                    .iter()
                    .zip(self.live_s_hz.iter())
                    .map(|(&t, &v)| [t, v])
                    .collect();

                let y_max = hz_max(&self.live_m_hz, &self.live_s_hz);

                Plot::new("hz_plot")
                    .legend(Legend::default())
                    .x_axis_label("Tiempo (s)")
                    .y_axis_label("Hz")
                    .allow_zoom(false)
                    .allow_scroll(false)
                    .allow_drag(false)
                    .allow_boxed_zoom(false)
                    .show(ui, |plot_ui| {
                        if !m_hz.is_empty() {
                            plot_ui.line(
                                Line::new(PlotPoints::new(m_hz))
                                    .color(C_ORANGE)
                                    .name("M Hz")
                                    .width(1.5f32),
                            );
                            plot_ui.line(
                                Line::new(PlotPoints::new(s_hz))
                                    .color(C_YELLOW)
                                    .name("S Hz")
                                    .width(1.5f32),
                            );
                        }
                        plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                            [t_min, 0.0],
                            [t_max, y_max],
                        ));
                    });
            });
        });
    }
}

// ============================================================================
// ToF CALIBRATION — RENDERING
// ============================================================================
impl SysIdApp {
    fn render_tof_tab(&mut self, ui: &mut egui::Ui) {
        let available = ui.available_size();
        let card_w = (available.x - 30.0) / 2.0;
        let card_h = (available.y - 10.0) / 2.0;

        ui.horizontal(|ui| {
            ui.add_space(5.0);
            ui.vertical(|ui| {
                // Row 1: Control + Detection
                ui.horizontal(|ui| {
                    ui.allocate_ui(egui::vec2(card_w, card_h), |ui| {
                        self.render_tof_control_card(ui);
                    });
                    ui.add_space(10.0);
                    ui.allocate_ui(egui::vec2(card_w, card_h), |ui| {
                        self.render_tof_detection_card(ui);
                    });
                });

                ui.add_space(8.0);

                // Row 2: Results + Transfer Function
                ui.horizontal(|ui| {
                    ui.allocate_ui(egui::vec2(card_w, card_h), |ui| {
                        self.render_tof_results_card(ui);
                    });
                    ui.add_space(10.0);
                    ui.allocate_ui(egui::vec2(card_w, card_h), |ui| {
                        self.render_tof_transfer_card(ui);
                    });
                });
            });
        });
    }

    fn render_tof_control_card(&mut self, ui: &mut egui::Ui) {
        let connected = self.serial.is_some();
        section(ui, "ToF — CONTROL", self.tof_state.color(), |ui| {
            // State indicator badge
            let state_color = self.tof_state.color();
            let state_label = self.tof_state.label();
            ui.horizontal(|ui| {
                ui.add_space(4.0);
                egui::Frame::none()
                    .fill(state_color)
                    .rounding(Rounding::same(6.0))
                    .inner_margin(egui::Margin::symmetric(16.0, 6.0))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new(state_label)
                                .strong()
                                .size(14.0)
                                .color(Color32::WHITE),
                        );
                    });
            });

            ui.add_space(8.0);

            let can_change_shooter = self.tof_state == TofState::Idle;
            ui.horizontal(|ui| {
                ui.label(RichText::new("Disparador:").size(10.0).color(FG_DIM));
                ui.add_enabled_ui(can_change_shooter, |ui| {
                    if ui
                        .selectable_label(self.tof_shooter == TofShooter::Master, "Master")
                        .clicked()
                    {
                        self.tof_shooter = TofShooter::Master;
                    }
                    if ui
                        .selectable_label(self.tof_shooter == TofShooter::Slave, "Slave")
                        .clicked()
                    {
                        self.tof_shooter = TofShooter::Slave;
                    }
                });
            });

            ui.add_space(4.0);

            let can_change_pulse = self.tof_state == TofState::Idle;
            ui.horizontal(|ui| {
                ui.label(RichText::new("Pulso (µs):").size(10.0).color(FG_DIM));
                ui.add_enabled_ui(can_change_pulse, |ui| {
                    ui.add(
                        egui::Slider::new(&mut self.tof_pulse_us, 1000..=2000)
                            .step_by(10.0)
                            .suffix(" µs"),
                    );
                });
            });

            ui.add_space(6.0);

            // Buttons
            ui.horizontal(|ui| {
                let can_arm =
                    connected && self.tof_state == TofState::Idle;
                let can_stop = connected
                    && self.tof_state != TofState::Idle
                    && self.tof_state != TofState::Done;
                let can_start =
                    connected && self.tof_state == TofState::Done;

                if ui
                    .add_enabled(can_arm, colored_button_widget("ARM", C_ARMING))
                    .clicked()
                {
                    self.tof_arm();
                }
                ui.add_space(4.0);
                if ui
                    .add_enabled(can_stop, colored_button_widget("STOP", C_RED))
                    .clicked()
                {
                    self.tof_stop();
                }
                ui.add_space(4.0);
                if ui
                    .add_enabled(
                        can_start,
                        colored_button_widget("▶ START", C_GREEN),
                    )
                    .clicked()
                {
                    self.tof_reset();
                    self.tof_arm();
                }
            });

            ui.add_space(6.0);

            // Status description
            if let Some(ref err) = self.tof_error_msg {
                ui.label(
                    RichText::new(err)
                        .size(11.0)
                        .strong()
                        .color(C_RED),
                );
            } else {
                let status_msg = match self.tof_state {
                    TofState::Idle => "Ready. Click ARM to begin.",
                    TofState::Arming => "Ramping ESC to target Hz...",
                    TofState::Silence => "Waiting for steady state...",
                    TofState::Firing => "Sniper pulse active (50ms)!",
                    TofState::Listening => "Listening for wave arrival...",
                    TofState::Done => {
                        if self.tof_wave_detected {
                            "Experiment complete. Wave detected!"
                        } else {
                            "Experiment complete."
                        }
                    }
                };
                ui.label(
                    RichText::new(status_msg)
                        .size(10.0)
                        .color(FG_DIM),
                );
            }
        });
    }

    fn render_tof_detection_card(&mut self, ui: &mut egui::Ui) {
        section(ui, "ToF — DETECCIÓN", C_CYAN, |ui| {
            // Threshold slider
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new("Umbral delta:")
                        .size(10.0)
                        .color(FG_DIM),
                );
                ui.add(
                    egui::Slider::new(&mut self.tof_threshold, 1.0..=100.0)
                        .show_value(true)
                        .trailing_fill(true),
                );
            });

            ui.add_space(4.0);

            // Heartbeat info
            let mono = egui::FontId::monospace(10.0);
            if let Some(t0) = self.tof_t0_ms {
                ui.label(
                    RichText::new(format!("T0: {}ms", t0))
                        .font(mono.clone())
                        .color(C_FIRING),
                );
            }
            if let Some(t1) = self.tof_t1_ms {
                ui.label(
                    RichText::new(format!("T1: {}ms", t1))
                        .font(mono.clone())
                        .color(C_LISTENING),
                );
            }
            if self.tof_tof_ms > 0.0 {
                ui.label(
                    RichText::new(format!("ToF: {:.2} ms", self.tof_tof_ms))
                        .font(mono.clone())
                        .color(C_BLUE)
                        .strong(),
                );
            }

            ui.add_space(4.0);

            // Live plot
            let plot_h = ui.available_height().max(80.0);
            let m_angle: Vec<[f64; 2]> = self
                .tof_times
                .iter()
                .zip(self.tof_m_angle.iter())
                .map(|(&t, &v)| [t, v])
                .collect();
            let s_angle: Vec<[f64; 2]> = self
                .tof_times
                .iter()
                .zip(self.tof_s_angle.iter())
                .map(|(&t, &v)| [t, v])
                .collect();

            ui.allocate_ui(egui::vec2(ui.available_width(), plot_h), |ui| {
                Plot::new("tof_angle_plot")
                    .legend(Legend::default())
                    .x_axis_label("Tiempo (s)")
                    .y_axis_label("Angle")
                    .allow_zoom(false)
                    .allow_scroll(false)
                    .allow_drag(false)
                    .allow_boxed_zoom(false)
                    .show(ui, |plot_ui| {
                        // Compute bounds first (before moving data into PlotPoints)
                        let (bounds_t_min, bounds_t_max, bounds_a_min, bounds_a_max) =
                            if !m_angle.is_empty() {
                                let t_min = self.tof_times.front().copied().unwrap_or(0.0);
                                let t_max = self.tof_times.back().copied().unwrap_or(t_min + 1.0);
                                let a_min = m_angle
                                    .iter()
                                    .chain(s_angle.iter())
                                    .map(|p| p[1])
                                    .fold(f64::INFINITY, f64::min);
                                let a_max = m_angle
                                    .iter()
                                    .chain(s_angle.iter())
                                    .map(|p| p[1])
                                    .fold(f64::NEG_INFINITY, f64::max);
                                let margin = (a_max - a_min).abs().max(10.0) * 0.2;
                                let a_min = if a_min.is_infinite() { -100.0 } else { a_min - margin };
                                let a_max = if a_max.is_infinite() { 100.0 } else { a_max + margin };
                                (t_min, f64::max(t_max, t_min + 1.0), a_min, a_max)
                            } else {
                                (0.0, 1.0, -100.0, 100.0)
                            };

                        if !m_angle.is_empty() {
                            plot_ui.line(
                                Line::new(PlotPoints::new(m_angle))
                                    .color(C_BLUE)
                                    .name("m_angle")
                                    .width(1.5f32),
                            );
                            plot_ui.line(
                                Line::new(PlotPoints::new(s_angle))
                                    .color(C_GREEN)
                                    .name("s_angle")
                                    .width(1.5f32),
                            );
                        }
                        // T0 marker
                        if let Some(t0) = self.tof_t0_line {
                            plot_ui.vline(
                                VLine::new(t0)
                                    .color(C_FIRING)
                                    .width(2.0f32)
                                    .style(LineStyle::dashed_loose())
                                    .name("T0"),
                            );
                        }
                        // T1 marker
                        if let Some(t1) = self.tof_t1_line {
                            plot_ui.vline(
                                VLine::new(t1)
                                    .color(C_LISTENING)
                                    .width(2.0f32)
                                    .style(LineStyle::dashed_loose())
                                    .name("T1"),
                            );
                        }
                        // Set bounds
                        plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                            [bounds_t_min, bounds_a_min],
                            [bounds_t_max, bounds_a_max],
                        ));
                    });
            });
        });
    }

    fn render_tof_results_card(&mut self, ui: &mut egui::Ui) {
        section(ui, "ToF — RESULTADOS", C_GREEN, |ui| {
            // Rope length input
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new("Largo cuerda:")
                        .size(10.0)
                        .color(FG_DIM),
                );
                let resp = ui.add(
                    egui::TextEdit::singleline(&mut self.tof_rope_entry)
                        .desired_width(60.0)
                        .font(egui::FontId::proportional(13.0))
                        .horizontal_align(egui::Align::Center),
                );
                if resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    if let Ok(v) = self.tof_rope_entry.parse::<f64>() {
                        self.tof_rope_length = v.clamp(0.5, 5.0);
                        self.tof_rope_entry = format!("{:.2}", self.tof_rope_length);
                    }
                }
                ui.label(RichText::new("m (0.5-5.0)").size(9.0).color(FG_DIM));
            });
            ui.add(
                egui::Slider::new(&mut self.tof_rope_length, 0.5..=5.0)
                    .show_value(true)
                    .trailing_fill(true),
            );
            if ui.input(|i| i.pointer.any_released()) {
                self.tof_rope_entry = format!("{:.2}", self.tof_rope_length);
            }

            ui.add_space(6.0);

            // Results
            let mono = egui::FontId::monospace(11.0);
            ui.label(
                RichText::new(format!("ToF:       {:.2} ms", self.tof_tof_ms))
                    .font(mono.clone())
                    .color(FG_TEXT),
            );
            ui.label(
                RichText::new(format!("Wavespeed: {:.4} m/s", self.tof_wavespeed))
                    .font(mono.clone())
                    .color(C_GREEN)
                    .strong(),
            );

            ui.add_space(6.0);

            // Export button
            if !self.tof_history.is_empty() {
                if ui
                    .add(colored_button_widget("Exportar CSV", C_GREEN))
                    .clicked()
                {
                    self.tof_export_csv();
                }
            }

            ui.add_space(6.0);

            ui.label(
                RichText::new("Historial:")
                    .strong()
                    .size(10.0)
                    .color(FG_DIM),
            );
            let table_mono = egui::FontId::monospace(9.0);
            let start = if self.tof_history.len() > 10 {
                self.tof_history.len() - 10
            } else {
                0
            };
            let visible: Vec<_> = self.tof_history[start..].to_vec();
            egui::ScrollArea::horizontal()
                .max_width(ui.available_width())
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label(RichText::new("Hora").font(table_mono.clone()).color(FG_DIM));
                        ui.label(RichText::new("Hz").font(table_mono.clone()).color(FG_DIM));
                        ui.label(RichText::new("ToF").font(table_mono.clone()).color(FG_DIM));
                        ui.label(RichText::new("V.m/s").font(table_mono.clone()).color(FG_DIM));
                    });
                    for r in &visible {
                        ui.horizontal(|ui| {
                            ui.label(
                                RichText::new(&r.timestamp).font(table_mono.clone()).color(FG_TEXT),
                            );
                            ui.label(
                                RichText::new(format!("{:.0}", r.target_hz))
                                    .font(table_mono.clone())
                                    .color(FG_TEXT),
                            );
                            ui.label(
                                RichText::new(format!("{:.1}", r.tof_ms))
                                    .font(table_mono.clone())
                                    .color(FG_TEXT),
                            );
                            ui.label(
                                RichText::new(format!("{:.2}", r.wavespeed))
                                    .font(table_mono.clone())
                                    .color(FG_TEXT),
                            );
                        });
                    }
                });
        });
    }

    fn render_tof_transfer_card(&mut self, ui: &mut egui::Ui) {
        section(ui, "ToF — FUNCIÓN DE TRANSFERENCIA", C_ORANGE, |ui| {
            let mono = egui::FontId::monospace(12.0);

            // Coefficient A
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new("A:")
                        .size(10.0)
                        .color(FG_DIM),
                );
                let resp = ui.add(
                    egui::TextEdit::singleline(&mut self.tof_a_entry)
                        .desired_width(80.0)
                        .font(egui::FontId::proportional(14.0))
                        .horizontal_align(egui::Align::Center),
                );
                if resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    if let Ok(v) = self.tof_a_entry.parse::<f64>() {
                        self.tof_coeff_a = v;
                        self.tof_a_entry = format!("{:.4}", self.tof_coeff_a);
                    }
                }
            });

            ui.add_space(4.0);

            // Coefficient B
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new("B:")
                        .size(10.0)
                        .color(FG_DIM),
                );
                let resp = ui.add(
                    egui::TextEdit::singleline(&mut self.tof_b_entry)
                        .desired_width(80.0)
                        .font(egui::FontId::proportional(14.0))
                        .horizontal_align(egui::Align::Center),
                );
                if resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    if let Ok(v) = self.tof_b_entry.parse::<f64>() {
                        self.tof_coeff_b = v;
                        self.tof_b_entry = format!("{:.4}", self.tof_coeff_b);
                    }
                }
            });

            ui.add_space(8.0);

            ui.label(
                RichText::new("Target Hz")
                    .size(10.0)
                    .color(FG_DIM),
            );
            ui.horizontal(|ui| {
                let resp = ui.add(
                    egui::TextEdit::singleline(&mut self.tof_entry_hz)
                        .desired_width(80.0)
                        .font(egui::FontId::proportional(14.0))
                        .horizontal_align(egui::Align::Center),
                );
                if resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    if let Ok(v) = self.tof_entry_hz.parse::<f64>() {
                        self.tof_target_hz = v.clamp(1.0, 100.0);
                        self.tof_entry_hz = format!("{:.0}", self.tof_target_hz);
                    }
                }
                ui.label(
                    RichText::new("Hz (1-100)")
                        .size(9.0)
                        .color(FG_DIM),
                );
            });
            ui.add(
                egui::Slider::new(&mut self.tof_target_hz, 1.0..=100.0)
                    .show_value(true)
                    .trailing_fill(true),
            );
            if ui.input(|i| i.pointer.any_released()) {
                self.tof_entry_hz = format!("{:.0}", self.tof_target_hz);
            }

            ui.add_space(4.0);

            let f1 = self.tof_target_hz;
            ui.label(
                RichText::new(format!("f1: {:.1} Hz", f1))
                    .font(mono.clone())
                    .color(FG_TEXT),
            );

            ui.add_space(4.0);

            // µs output
            let us_output = if self.tof_coeff_a != 0.0 {
                (f1 - self.tof_coeff_b) / self.tof_coeff_a
            } else {
                0.0
            };
            ui.label(
                RichText::new(format!("µs output: {:.1}", us_output))
                    .font(mono.clone())
                    .color(C_ORANGE)
                    .strong(),
            );

            ui.add_space(6.0);

            // Formula display
            ui.label(
                RichText::new(format!(
                    "µs = (f1 - B) / A = ({:.1} - {:.2}) / {:.4}",
                    f1, self.tof_coeff_b, self.tof_coeff_a
                ))
                .size(9.0)
                .color(FG_DIM),
            );

            ui.add_space(8.0);

            // Send to ESC button
            let connected = self.serial.is_some();
            let us_val = us_output.round() as i16;
            if ui
                .add_enabled(
                    connected,
                    colored_button_widget(
                        &format!("Enviar {}µs a ESC", us_val),
                        C_ORANGE,
                    ),
                )
                .clicked()
            {
                if let Some(ref mut serial) = self.serial {
                    let _ = serial.send_command(CMD_TOF_START, us_val.clamp(1000, 2000));
                }
                self.status_text = format!("TOF: Enviado {}µs a ESC", us_val);
                self.status_color = C_ORANGE;
            }
        });
    }
}

// ============================================================================
// eframe::App
// ============================================================================
impl eframe::App for SysIdApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Cargar logo una vez
        if !self.logo_loaded {
            self.load_logo(ctx);
        }

        // Procesar datos serial entrantes
        self.poll_serial();
        self.check_watchdog();

        // ToF state machine
        self.tof_update_state();

        // Cierre diferido (stop_all -> cerrar en 500ms)
        if let Some(t) = self.close_after {
            if Instant::now() >= t {
                if let Some(ref mut serial) = self.serial {
                    let _ = serial.send_command(CMD_STOP_ALL, 0);
                    serial.close();
                }
                self.serial = None;
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                return;
            }
        }

        // Repintar continuamente si conectado (gráficos en vivo)
        if self.serial.is_some() {
            ctx.request_repaint();
        }

        // ---- HEADER ----
        egui::TopBottomPanel::top("header")
            .frame(
                egui::Frame::none()
                    .fill(BG_MAIN)
                    .inner_margin(egui::Margin::symmetric(15.0, 10.0))
                    .stroke(Stroke::new(1.0f32, C_BORDER)),
            )
            .exact_height(70.0)
            .show(ctx, |ui| {
                self.render_header(ui);
            });

        // ---- TAB BAR ----
        egui::TopBottomPanel::top("tab_bar")
            .frame(
                egui::Frame::none()
                    .fill(BG_MAIN)
                    .inner_margin(egui::Margin::symmetric(5.0, 4.0))
                    .stroke(Stroke::new(1.0f32, C_BORDER)),
            )
            .exact_height(36.0)
            .show(ctx, |ui| {
                self.render_tab_bar(ui);
            });

        // ---- CONTENT BASED ON TAB ----
        match self.active_tab {
            Tab::Control => {
                // ---- PANEL IZQUIERDO ----
                egui::SidePanel::left("controls")
                    .exact_width(SIDE_PANEL_WIDTH)
                    .frame(
                        egui::Frame::none()
                            .fill(BG_MAIN)
                            .inner_margin(egui::Margin::symmetric(12.0, 10.0))
                            .stroke(Stroke::new(1.0f32, C_BORDER)),
                    )
                    .show(ctx, |ui| {
                        egui::ScrollArea::vertical()
                            .auto_shrink([false; 2])
                            .show(ui, |ui| {
                                self.render_controls(ui);
                            });
                    });

                // ---- PANEL CENTRAL (GRÁFICOS) ----
                egui::CentralPanel::default()
                    .frame(
                        egui::Frame::none()
                            .fill(BG_MAIN)
                            .inner_margin(egui::Margin::symmetric(10.0, 5.0)),
                    )
                    .show(ctx, |ui| {
                        self.render_plots(ui);
                    });
            }
            Tab::TofCalibration => {
                // ---- ToF CALIBRATION: Full-width panel ----
                egui::CentralPanel::default()
                    .frame(
                        egui::Frame::none()
                            .fill(BG_MAIN)
                            .inner_margin(egui::Margin::symmetric(5.0, 5.0)),
                    )
                    .show(ctx, |ui| {
                        self.render_tof_tab(ui);
                    });
            }
        }

        // ---- MODAL DE MENSAJE ----
        let mut close_msg = false;
        if let Some((ref title, ref body)) = self.message {
            egui::Window::new(title.as_str())
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
                .frame(
                    egui::Frame::window(&ctx.style())
                        .fill(BG_MAIN)
                        .rounding(Rounding::same(8.0))
                        .stroke(Stroke::new(1.0f32, C_BORDER)),
                )
                .show(ctx, |ui| {
                    ui.add_space(5.0);
                    ui.label(RichText::new(body.as_str()).color(FG_TEXT));
                    ui.add_space(10.0);
                    if colored_button(ui, "OK", C_BLUE) {
                        close_msg = true;
                    }
                });
        }
        if close_msg {
            self.message = None;
        }
    }
}

// ============================================================================
// HELPERS
// ============================================================================

fn push_buf(buf: &mut VecDeque<f64>, val: f64) {
    if buf.len() >= LIVE_BUFFER_SIZE {
        buf.pop_front();
    }
    buf.push_back(val);
}

fn push_buf_sized(buf: &mut VecDeque<f64>, val: f64, max_size: usize) {
    if buf.len() >= max_size {
        buf.pop_front();
    }
    buf.push_back(val);
}

/// Sección estilizada (similar a LabelFrame de Tkinter)
fn section(ui: &mut egui::Ui, title: &str, color: Color32, add_body: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::none()
        .fill(BG_PANEL)
        .stroke(Stroke::new(1.0f32, C_BORDER))
        .rounding(Rounding::same(6.0))
        .inner_margin(egui::Margin::same(12.0))
        .show(ui, |ui| {
            ui.label(RichText::new(title).strong().size(10.0).color(color));
            ui.add_space(6.0);
            add_body(ui);
        });
}

/// Crea un widget Button con estilo flat y color de fondo
fn colored_button_widget(text: &str, color: Color32) -> egui::Button<'_> {
    egui::Button::new(RichText::new(text).color(Color32::WHITE).strong().size(11.0))
        .fill(color)
        .rounding(Rounding::same(4.0))
        .min_size(egui::vec2(0.0, 32.0))
}

/// Atajo: crear y agregar un botón coloreado, retorna true si se hizo clic
fn colored_button(ui: &mut egui::Ui, text: &str, color: Color32) -> bool {
    ui.add(colored_button_widget(text, color)).clicked()
}

// ---- Cálculo de bounds para cada gráfico ----

fn dc_bounds(m_pwm: &VecDeque<f64>, s_pwm: &VecDeque<f64>) -> (f64, f64) {
    let pmin = m_pwm
        .iter()
        .chain(s_pwm.iter())
        .copied()
        .fold(f64::INFINITY, f64::min);
    let pmax = m_pwm
        .iter()
        .chain(s_pwm.iter())
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);
    let pmin = if pmin.is_infinite() { -1.0 } else { pmin };
    let pmax = if pmax.is_infinite() { 1.0 } else { pmax };
    let margin = f64::max(pmin.abs(), f64::max(pmax.abs(), 1.0)) * 0.15;
    (pmin - margin, pmax + margin)
}

fn ac_bounds(m_esc: &VecDeque<f64>, s_esc: &VecDeque<f64>) -> (f64, f64) {
    let emin = m_esc
        .iter()
        .chain(s_esc.iter())
        .copied()
        .fold(f64::INFINITY, f64::min);
    let emax = m_esc
        .iter()
        .chain(s_esc.iter())
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);
    let emin = if emin.is_infinite() { 1000.0 } else { emin };
    let emax = if emax.is_infinite() { 2000.0 } else { emax };
    let em = f64::max((emax - emin) * 0.1, 10.0);
    (emin - em, emax + em)
}

fn rpm_max(m_rpm: &VecDeque<f64>, s_rpm: &VecDeque<f64>) -> f64 {
    let rmax = m_rpm
        .iter()
        .chain(s_rpm.iter())
        .copied()
        .map(f64::abs)
        .fold(0.0_f64, f64::max);
    let rmax = if rmax == 0.0 { 1.0 } else { rmax };
    f64::max(rmax * 1.2, 1.0)
}

fn hz_max(m_hz: &VecDeque<f64>, s_hz: &VecDeque<f64>) -> f64 {
    let hmax = m_hz
        .iter()
        .chain(s_hz.iter())
        .copied()
        .fold(0.0_f64, f64::max);
    let hmax = if hmax == 0.0 { 1.0 } else { hmax };
    f64::max(hmax * 1.2, 1.0)
}
