mod config;
mod effects;
mod hid;
mod state;
mod xplane;

use anyhow::Result;
use config::{Config, ConnectionPref};
use effects::EffectMixer;
use hid::{Connection, DualSense, OutputState, TriggerEffect};
use state::FlightState;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use xplane::XPlaneClient;

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let config_path = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("config.toml"));
    let config = Config::load(&config_path)?;
    log::info!("loaded config: {config:?}");

    let running = Arc::new(AtomicBool::new(true));
    {
        let running = running.clone();
        ctrlc::set_handler(move || {
            log::info!("shutdown requested");
            running.store(false, Ordering::SeqCst);
        })?;
    }

    // --- HID: open DualSense, Bluetooth preferred per project priority -----
    let api = hidapi::HidApi::new()?;
    let prefer = match config.connection {
        ConnectionPref::Bluetooth => Some(Connection::Bluetooth),
        ConnectionPref::Usb => Some(Connection::Usb),
        ConnectionPref::Auto => None,
    };
    let mut pad = DualSense::open(&api, prefer)?;
    log::info!("connected to DualSense over {:?}", pad.connection);

    // --- X-Plane: subscribe to all datarefs we care about -------------------
    let xp = XPlaneClient::new(
        &config.xplane_ip,
        config.xplane_port,
        config.local_port,
        Duration::from_millis(20),
    )?;
    xp.subscribe_all(state::SUBSCRIPTIONS)?;
    log::info!(
        "subscribed to {} datarefs from X-Plane at {}:{}",
        state::SUBSCRIPTIONS.len(),
        config.xplane_ip,
        config.xplane_port
    );

    let flight_state = Arc::new(Mutex::new(FlightState::new()));

    // --- UDP receive thread: keep flight_state fresh ------------------------
    {
        let flight_state = flight_state.clone();
        let running = running.clone();
        std::thread::spawn(move || {
            while running.load(Ordering::SeqCst) {
                match xp.recv() {
                    Ok(pairs) => {
                        if !pairs.is_empty() {
                            let mut fs = flight_state.lock().unwrap();
                            for (idx, val) in pairs {
                                fs.apply(idx, val);
                            }
                        }
                    }
                    Err(e) => {
                        log::warn!("xplane recv error: {e}");
                        std::thread::sleep(Duration::from_millis(200));
                    }
                }
            }
        });
    }

    // --- Effect mixer + HID write loop (main thread) -------------------------
    let mixer = EffectMixer::new(config.gains.clone());
    let tick = Duration::from_millis((1000 / config.tick_hz.max(1)) as u64);
    log::info!("running effect loop at {} Hz", config.tick_hz);

    while running.load(Ordering::SeqCst) {
        let (left, right, lightbar) = {
            let fs = flight_state.lock().unwrap();
            mixer.compute(&fs)
        };

        let out = OutputState {
            motor_left: left,
            motor_right: right,
            lightbar,
            player_leds: 0b00100,
        };

        if let Err(e) = pad.write(out, TriggerEffect::Off, TriggerEffect::Off) {
            log::warn!("hid write failed: {e}");
        }

        std::thread::sleep(tick);
    }

    let _ = pad.write(OutputState::default(), TriggerEffect::Off, TriggerEffect::Off);
    log::info!("shut down cleanly");
    Ok(())
}
