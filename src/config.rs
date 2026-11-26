use serde::Deserialize;
use std::path::{Path, PathBuf};

#[derive(Debug, Deserialize, Clone, Copy)]
#[serde(rename_all = "snake_case")]
pub enum ConnectionPref {
    Auto,
    Bluetooth,
    Usb,
}

#[derive(Debug, Deserialize, Clone)]
pub struct Gains {
    #[serde(default = "d_ground")]
    pub ground_rumble: f32,
    #[serde(default = "d_touchdown")]
    pub touchdown: f32,
    #[serde(default = "d_stall")]
    pub stall_buffet: f32,
    #[serde(default = "d_turb")]
    pub turbulence: f32,
    #[serde(default = "d_overspeed")]
    pub overspeed: f32,
    #[serde(default = "d_gear")]
    pub gear_flap: f32,
    #[serde(default = "d_engine")]
    pub engine_rumble: f32,
    #[serde(default = "d_speedbrake")]
    pub speedbrake_buffet: f32,
    #[serde(default = "d_surface")]
    pub surface_rough: f32,
    #[serde(default = "d_gear_buffet")]
    pub gear_buffet: f32,
    #[serde(default = "d_engine_start")]
    pub engine_start: f32,
    #[serde(default = "d_rotor_beat")]
    pub rotor_beat: f32,
    #[serde(default = "d_prop_beat")]
    pub prop_beat: f32,
    #[serde(default = "d_ap_disc")]
    pub ap_disconnect: f32,
    #[serde(default = "d_config_overspeed")]
    pub config_overspeed: f32,
    #[serde(default = "d_nose_touchdown")]
    pub nose_touchdown: f32,
}

fn d_ground() -> f32 {
    0.35
}
fn d_touchdown() -> f32 {
    1.0
}
fn d_stall() -> f32 {
    0.8
}
fn d_turb() -> f32 {
    0.5
}
fn d_overspeed() -> f32 {
    0.9
}
fn d_gear() -> f32 {
    0.4
}
fn d_engine() -> f32 {
    0.0
} // off by default -- opt in if you want a faint idle hum
fn d_speedbrake() -> f32 {
    0.5
}
fn d_surface() -> f32 {
    0.35
}
fn d_gear_buffet() -> f32 {
    0.25
}
fn d_engine_start() -> f32 {
    0.6
}
fn d_rotor_beat() -> f32 {
    0.25
}
fn d_prop_beat() -> f32 {
    0.0
} // off by default for the same reason as engine_rumble
fn d_ap_disc() -> f32 {
    0.6
}
fn d_config_overspeed() -> f32 {
    0.6
}
fn d_nose_touchdown() -> f32 {
    0.6
}

impl Default for Gains {
    fn default() -> Self {
        Self {
            ground_rumble: d_ground(),
            touchdown: d_touchdown(),
            stall_buffet: d_stall(),
            turbulence: d_turb(),
            overspeed: d_overspeed(),
            gear_flap: d_gear(),
            engine_rumble: d_engine(),
            speedbrake_buffet: d_speedbrake(),
            surface_rough: d_surface(),
            gear_buffet: d_gear_buffet(),
            engine_start: d_engine_start(),
            rotor_beat: d_rotor_beat(),
            prop_beat: d_prop_beat(),
            ap_disconnect: d_ap_disc(),
            config_overspeed: d_config_overspeed(),
            nose_touchdown: d_nose_touchdown(),
        }
    }
}

/// Adaptive trigger behaviour. Detent zones are trigger-travel positions
/// (0 = released, 9 = fully pulled), so they line up with throttle settings
/// only if R2 is bound to the throttle axis in X-Plane.
#[derive(Debug, Deserialize, Clone)]
pub struct TriggerConfig {
    /// Master switch. false = never touch the triggers.
    #[serde(default = "d_true")]
    pub enabled: bool,
    /// R2 throttle: light resistance with a detent (resistance step).
    #[serde(default = "d_true")]
    pub throttle: bool,
    /// Zone (1-9) where the detent starts, e.g. 8 ~ climb/TOGA gate.
    #[serde(default = "d_detent_zone")]
    pub throttle_detent_zone: u8,
    /// Resistance (1-8) from the detent onward. Below it R2 stays at 1.
    #[serde(default = "d_detent_strength")]
    pub throttle_detent_strength: u8,
    /// R2 buzzes while thrust reversers are deployed.
    #[serde(default = "d_true")]
    pub reverse_buzz: bool,
    /// L2 brakes: firmer with groundspeed, locked solid with parking brake set.
    #[serde(default = "d_true")]
    pub brakes: bool,
    /// Both triggers shake while the stall warning is active.
    #[serde(default = "d_true")]
    pub stall_shaker: bool,
    /// Stick-shaker vibration frequency in Hz.
    #[serde(default = "d_shaker_hz")]
    pub stall_shaker_hz: u8,
}

fn d_true() -> bool {
    true
}
fn d_detent_zone() -> u8 {
    8
}
fn d_detent_strength() -> u8 {
    6
}
fn d_shaker_hz() -> u8 {
    20
}

impl Default for TriggerConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            throttle: true,
            throttle_detent_zone: d_detent_zone(),
            throttle_detent_strength: d_detent_strength(),
            reverse_buzz: true,
            brakes: true,
            stall_shaker: true,
            stall_shaker_hz: d_shaker_hz(),
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
pub struct Config {
    #[serde(default = "d_xplane_ip")]
    pub xplane_ip: String,
    #[serde(default = "d_xplane_port")]
    pub xplane_port: u16,
    #[serde(default = "d_local_port")]
    pub local_port: u16,
    #[serde(default = "d_connection_pref")]
    pub connection: ConnectionPref,
    #[serde(default = "d_tick_hz")]
    pub tick_hz: u32,
    /// Override the stall AoA (degrees). Unset = use the aircraft's own
    /// acf_stall_warn_alpha from X-Plane, falling back to 14 degrees.
    #[serde(default)]
    pub stall_alpha_deg: Option<f32>,
    #[serde(default)]
    pub gains: Gains,
    #[serde(default)]
    pub triggers: TriggerConfig,
}

fn d_xplane_ip() -> String {
    "127.0.0.1".to_string()
}
fn d_xplane_port() -> u16 {
    49000
}
fn d_local_port() -> u16 {
    49010
}
fn d_connection_pref() -> ConnectionPref {
    ConnectionPref::Bluetooth
}
fn d_tick_hz() -> u32 {
    60
}

impl Default for Config {
    fn default() -> Self {
        Self {
            xplane_ip: d_xplane_ip(),
            xplane_port: d_xplane_port(),
            local_port: d_local_port(),
            connection: d_connection_pref(),
            tick_hz: d_tick_hz(),
            stall_alpha_deg: None,
            gains: Gains::default(),
            triggers: TriggerConfig::default(),
        }
    }
}

impl Config {
    /// Resolve which config file to use: an explicit CLI path wins; else
    /// `config.toml` in the working directory; else `config.toml` next to the
    /// executable (so double-clicking the .exe on Windows finds its config).
    pub fn resolve_path(cli_arg: Option<PathBuf>) -> PathBuf {
        if let Some(p) = cli_arg {
            return p;
        }
        let cwd = PathBuf::from("config.toml");
        if cwd.exists() {
            return cwd;
        }
        if let Some(beside_exe) = std::env::current_exe().ok().and_then(|e| e.parent().map(|d| d.join("config.toml"))) {
            if beside_exe.exists() {
                return beside_exe;
            }
        }
        cwd
    }

    pub fn load(path: &Path) -> anyhow::Result<Self> {
        if !path.exists() {
            log::warn!("config file {path:?} not found, using built-in defaults");
            return Ok(Self::default());
        }
        let text = std::fs::read_to_string(path)?;
        let cfg: Config = toml::from_str(&text)?;
        Ok(cfg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shipped_config_parses_and_sets_every_field() {
        let text = include_str!("../config.toml");
        let cfg: Config = toml::from_str(text).expect("config.toml must parse");
        assert!(cfg.triggers.enabled);
        assert_eq!(cfg.gains.prop_beat, 0.0);
    }

    #[test]
    fn empty_config_uses_defaults() {
        let cfg: Config = toml::from_str("").unwrap();
        assert_eq!(cfg.triggers.throttle_detent_zone, 8);
        assert!(cfg.stall_alpha_deg.is_none());
    }
}
