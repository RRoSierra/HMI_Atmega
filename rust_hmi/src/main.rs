mod app;
mod protocol;
mod serial_comm;

use eframe::egui;

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 850.0])
            .with_title("Atacama Dynamics - DAQ Identificación v4.0 (Rust)"),
        ..Default::default()
    };

    eframe::run_native(
        "Atacama Dynamics HMI",
        options,
        Box::new(|cc| Ok(Box::new(app::SysIdApp::new(cc)))),
    )
}
