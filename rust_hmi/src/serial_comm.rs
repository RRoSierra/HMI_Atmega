// ============================================================================
// Comunicación Serial - Hilo de lectura y gestión de conexión
// ============================================================================

use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::Duration;

use crate::protocol::{self, TelemetryPacket, PKT_SIZE, PKT_START};

/// Eventos enviados desde el hilo serial hacia la UI
pub enum SerialEvent {
    Packet(TelemetryPacket),
    ParseError,
    Disconnected,
}

/// Conexión serial activa
pub struct SerialConnection {
    writer: Box<dyn serialport::SerialPort>,
    pub receiver: mpsc::Receiver<SerialEvent>,
    running: Arc<AtomicBool>,
}

impl SerialConnection {
    pub fn open(port_name: &str) -> Result<Self, serialport::Error> {
        let port = serialport::new(port_name, 460800)
            .timeout(Duration::from_millis(100))
            .open()?;

        let reader = port.try_clone()?;
        let running = Arc::new(AtomicBool::new(true));
        let (tx, rx) = mpsc::channel();

        let running_clone = running.clone();
        std::thread::spawn(move || {
            Self::reader_thread(reader, tx, running_clone);
        });

        Ok(SerialConnection {
            writer: port,
            receiver: rx,
            running,
        })
    }

    fn reader_thread(
        mut reader: Box<dyn serialport::SerialPort>,
        tx: mpsc::Sender<SerialEvent>,
        running: Arc<AtomicBool>,
    ) {
        let mut buf = Vec::with_capacity(2048);
        let mut read_buf = [0u8; 512];

        while running.load(Ordering::Relaxed) {
            match reader.read(&mut read_buf) {
                Ok(n) if n > 0 => {
                    buf.extend_from_slice(&read_buf[..n]);

                    // Procesar todos los paquetes completos en el buffer
                    while buf.len() >= PKT_SIZE {
                        // Buscar byte de inicio 0xAA
                        if let Some(idx) = buf.iter().position(|&b| b == PKT_START) {
                            if idx > 0 {
                                // Descartar bytes basura antes del 0xAA
                                for _ in 0..idx {
                                    let _ = tx.send(SerialEvent::ParseError);
                                }
                                buf.drain(..idx);
                            }
                            if buf.len() < PKT_SIZE {
                                break;
                            }

                            // Extraer 22 bytes candidatos
                            let frame: [u8; PKT_SIZE] =
                                buf[..PKT_SIZE].try_into().unwrap();

                            if let Some(pkt) = TelemetryPacket::parse(&frame) {
                                buf.drain(..PKT_SIZE);
                                if tx.send(SerialEvent::Packet(pkt)).is_err() {
                                    return;
                                }
                            } else {
                                // Checksum falló: descartar el 0xAA y buscar siguiente
                                let _ = tx.send(SerialEvent::ParseError);
                                buf.drain(..1);
                            }
                        } else {
                            buf.clear();
                            break;
                        }
                    }
                }
                Ok(_) => {} // timeout, sin datos
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(_) => {
                    let _ = tx.send(SerialEvent::Disconnected);
                    break;
                }
            }
        }
    }

    /// Envía un comando de 3 bytes por serial
    pub fn send_command(&mut self, cmd: u8, val: i16) -> Result<(), std::io::Error> {
        let bytes = protocol::encode_command(cmd, val);
        self.writer.write_all(&bytes)?;
        self.writer.flush()
    }

    pub fn close(&mut self) {
        self.running.store(false, Ordering::Relaxed);
    }
}

impl Drop for SerialConnection {
    fn drop(&mut self) {
        self.close();
    }
}

/// Lista los puertos seriales disponibles
pub fn list_ports() -> Vec<String> {
    serialport::available_ports()
        .unwrap_or_default()
        .into_iter()
        .map(|p| p.port_name)
        .collect()
}
