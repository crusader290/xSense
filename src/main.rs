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
use std::time::{Duration, Instant};
use xplane::XPlaneClient;

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args: Vec<String> = std::env::args().skip(1).collect();
    let test_mode = args.iter().any(|a| a == "--test");
    let config_path = Config::resolve_path(args.iter().find(|a| !a.starts_with("--")).map(PathBuf::from));
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

    if test_mode {
        return controller_self_test(&mut pad);
    }

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
    // X-Plane forgets RREF subscriptions when it restarts, and never sees
    // them if it wasn't listening when we started, so re-send them every
    // RESUBSCRIBE_EVERY until data flows again.
    {
        let xp = xp.clone();
        let subs = subs.clone();
        let flight_state = flight_state.clone();
        let running = running.clone();
        std::thread::spawn(move || {
            const RESUBSCRIBE_EVERY: Duration = Duration::from_secs(2);
            let mut last_rx: Option<Instant> = None;
            let mut last_sub = Instant::now();
            let mut waiting_logged = false;
            while running.load(Ordering::SeqCst) {
                match xp.recv() {
                    Ok(pairs) => {
                        if !pairs.is_empty() {
                            if last_rx.is_none() || waiting_logged {
                                log::info!("receiving data from X-Plane");
                                waiting_logged = false;
                            }
                            last_rx = Some(Instant::now());
                            let mut fs = flight_state.lock().unwrap();
                            for (idx, val) in pairs {
                                fs.apply(idx, val);
                            }
                        }
                    }
                    Err(e) => {
                        log::warn!("xplane recv error: {e:#}");
                        std::thread::sleep(Duration::from_millis(200));
                    }
                }

                let silent = last_rx.is_none_or(|t| t.elapsed() > RESUBSCRIBE_EVERY);
                if silent && last_sub.elapsed() > RESUBSCRIBE_EVERY {
                    if !waiting_logged {
                        log::info!("waiting for X-Plane (is it running with a flight loaded?) - resubscribing every {}s", RESUBSCRIBE_EVERY.as_secs());
                        waiting_logged = true;
                    }
                    if let Err(e) = xp.subscribe_all(&subs) {
                        log::debug!("resubscribe failed: {e:#}");
                    }
                    last_sub = Instant::now();
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
                    log::warn!("hid write failed ({consecutive_failures}x): {e:#}");
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

/// `--test`: exercise every output without X-Plane, so a user can tell a
/// controller-side problem from an X-Plane/network one.
fn controller_self_test(pad: &mut DualSense) -> Result<()> {
    let steps: [(&str, OutputState, Option<TriggerEffect>, Option<TriggerEffect>); 6] = [
        ("lightbar red", OutputState { lightbar: (255, 0, 0), player_leds: 0b00100, ..Default::default() }, None, None),
        ("left (heavy) motor", OutputState { motor_left: 200, lightbar: (0, 255, 0), player_leds: 0b01010, ..Default::default() }, None, None),
        ("right (light) motor", OutputState { motor_right: 200, lightbar: (0, 0, 255), player_leds: 0b10001, ..Default::default() }, None, None),
        (
            "R2 resistance - pull it",
            OutputState { lightbar: (255, 255, 0), player_leds: 0b00100, ..Default::default() },
            Some(TriggerEffect::Feedback { start: 2, strength: 8 }),
            Some(TriggerEffect::Off),
        ),
        (
            "L2 resistance - pull it",
            OutputState { lightbar: (0, 255, 255), player_leds: 0b00100, ..Default::default() },
            Some(TriggerEffect::Off),
            Some(TriggerEffect::Feedback { start: 2, strength: 8 }),
        ),
        (
            "both triggers vibrating",
            OutputState { lightbar: (255, 0, 255), player_leds: 0b11111, ..Default::default() },
            Some(TriggerEffect::Vibration { start: 0, amplitude: 8, frequency: 20 }),
            Some(TriggerEffect::Vibration { start: 0, amplitude: 8, frequency: 20 }),
        ),
    ];
    for (name, out, rt, lt) in steps {
        log::info!("self-test: {name}");
        // Re-send for ~2.5s, like the real loop does at 60 Hz.
        for _ in 0..150 {
            if let Err(e) = pad.write(out, rt, lt) {
                log::error!("self-test write failed: {e:#}");
                return Err(e);
            }
            std::thread::sleep(Duration::from_millis(16));
        }
    }
    pad.write(OutputState::default(), Some(TriggerEffect::Off), Some(TriggerEffect::Off))?;
    log::info!("self-test done");
    Ok(())
}
