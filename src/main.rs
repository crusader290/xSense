mod config;
mod effects;
mod hid;
mod state;
mod xplane;

use anyhow::Result;
use config::{Config, ConnectionPref};
use effects::EffectMixer;
use hid::{Connection, DualSense, OutputState, TriggerEffect};
use state::subscriptions;
use state::FlightState;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use xplane::XPlaneClient;

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let config_path = Config::resolve_path(std::env::args().nth(1).map(PathBuf::from));
    log::info!("config path: {}", config_path.display());
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
    let xp = Arc::new(XPlaneClient::new(
        &config.xplane_ip,
        config.xplane_port,
        config.local_port,
        Duration::from_millis(20),
    )?);
    let subs = subscriptions();
    xp.subscribe_all(&subs)?;
    log::info!(
        "subscribed to {} datarefs from X-Plane at {}:{}",
        subs.len(),
        config.xplane_ip,
        config.xplane_port
    );

    let flight_state = Arc::new(Mutex::new(FlightState::new()));

    // --- UDP receive thread: keep flight_state fresh ------------------------
    {
        let xp = xp.clone();
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
    let mut mixer = EffectMixer::new(config.gains.clone(), config.triggers.clone(), config.stall_alpha_deg);
    let tick = Duration::from_millis((1000 / config.tick_hz.max(1)) as u64);
    log::info!("running effect loop at {} Hz", config.tick_hz);

    let mut consecutive_failures = 0u32;
    while running.load(Ordering::SeqCst) {
        let mix = {
            let fs = flight_state.lock().unwrap();
            mixer.compute(&fs)
        };

        let out = OutputState {
            motor_left: mix.left,
            motor_right: mix.right,
            lightbar: mix.lightbar,
            player_leds: 0b00100,
        };

        match pad.write(out, mix.right_trigger, mix.left_trigger) {
            Ok(()) => consecutive_failures = 0,
            Err(e) => {
                consecutive_failures += 1;
                // Don't spam 60 warnings/s if the pad is disconnected.
                if consecutive_failures == 1 || consecutive_failures.is_multiple_of(300) {
                    log::warn!("hid write failed ({consecutive_failures}x): {e}");
                }
            }
        }

        std::thread::sleep(tick);
    }

    xp.unsubscribe_all(&subs);
    let release = config.triggers.enabled.then_some(TriggerEffect::Off);
    let _ = pad.write(OutputState::default(), release, release);
    log::info!("shut down cleanly");
    Ok(())
}
