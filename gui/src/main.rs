//! BatterySense GUI
//!
//! Tauri + Leptos desktop application for monitoring BatterySense BMS data.
//!
//! This is a scaffold for the GUI. Full frontend (Leptos components) lives
//! in `gui/src/frontend/` and would be built with `cargo tauri build`.
//!
//! Usage:
//!   cargo run -p batterysense-gui

use tauri::Manager;

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .plugin(tauri_plugin_log::Builder::default().build())
        .invoke_handler(tauri::generate_handler![greet])
        .setup(|app| {
            #[cfg(debug_assertions)]
            {
                let window = app.get_window("main").unwrap();
                window.open_devtools();
            }
            Ok(())
        })
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}

#[tauri::command]
fn greet(name: &str) -> String {
    format!("Hello, {}! BatterySense GUI is running.", name)
}

fn main() {
    run();
}
