// ============================================================================
// Banco de Identificación de Sistemas - HMI Desktop v4.0 (Rust/egui)
// Atacama Dynamics
//
// Equivalente funcional del HMI en Python/Tkinter, usando egui para
// renderizado inmediato con mayor fluidez en gráficos en tiempo real.
// ============================================================================

use std::collections::VecDeque;
use std::time::Instant;

use eframe::egui;
use egui::{Color32, RichText, Rounding, Stroke};
use egui_plot::{Legend, Line, Plot, PlotBounds, PlotPoints};

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
        visuals.widgets.noninteractive.fg_stroke = Stroke::new(1.0, FG_TEXT);
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

                    // CSV
                    if self.is_recording {
                        self.data_log.push(pkt.clone());
                    }

                    self.last_packet = Some(pkt);
                }
                SerialEvent::ParseError => {
                    self.pkt_errors += 1;
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
        if self.is_recording {
            self.stop_recording();
        }
        self.close_after = Some(Instant::now() + std::time::Duration::from_millis(500));
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
                                    .width(1.5),
                            );
                            plot_ui.line(
                                Line::new(PlotPoints::new(s_pwm))
                                    .color(C_GREEN)
                                    .name("S PWM")
                                    .width(1.5),
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
                                    .width(1.5),
                            );
                            plot_ui.line(
                                Line::new(PlotPoints::new(s_esc))
                                    .color(C_YELLOW)
                                    .name("S ESC")
                                    .width(1.5),
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
                                    .width(1.5),
                            );
                            plot_ui.line(
                                Line::new(PlotPoints::new(s_rpm))
                                    .color(C_GREEN)
                                    .name("S RPM")
                                    .width(1.5),
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
                                    .width(1.5),
                            );
                            plot_ui.line(
                                Line::new(PlotPoints::new(s_hz))
                                    .color(C_YELLOW)
                                    .name("S Hz")
                                    .width(1.5),
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
                    .stroke(Stroke::new(1.0, C_BORDER)),
            )
            .exact_height(70.0)
            .show(ctx, |ui| {
                self.render_header(ui);
            });

        // ---- PANEL IZQUIERDO ----
        egui::SidePanel::left("controls")
            .exact_width(SIDE_PANEL_WIDTH)
            .frame(
                egui::Frame::none()
                    .fill(BG_MAIN)
                    .inner_margin(egui::Margin::symmetric(12.0, 10.0))
                    .stroke(Stroke::new(1.0, C_BORDER)),
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
                        .stroke(Stroke::new(1.0, C_BORDER)),
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

/// Sección estilizada (similar a LabelFrame de Tkinter)
fn section(ui: &mut egui::Ui, title: &str, color: Color32, add_body: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::none()
        .fill(BG_PANEL)
        .stroke(Stroke::new(1.0, C_BORDER))
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
