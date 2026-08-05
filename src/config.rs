use serde::Deserialize;
use std::path::Path;

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
    #[serde(default)]
    pub gains: Gains,
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
            gains: Gains::default(),
        }
    }
}

impl Config {
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
