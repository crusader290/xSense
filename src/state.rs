//! Dataref registry + the live flight state the effect mixer reads from.
//!
//! Each dataref we care about gets a fixed integer index (used as the RREF
//! subscription tag), so incoming (index, value) pairs can be routed with a
//! plain match instead of string comparisons on the hot path. Array datarefs
//! get a contiguous block of indices, one per element we subscribe to.

use std::time::{Duration, Instant};

// --- Dataref index assignments -------------------------------------------
pub const IDX_GROUNDSPEED: i32 = 1; // sim/flightmodel/position/groundspeed (m/s)
pub const IDX_VH_IND_FPM: i32 = 2; // sim/flightmodel/position/vh_ind_fpm
pub const IDX_ONGROUND_ANY: i32 = 3; // sim/flightmodel/failures/onground_any (0/1)
pub const IDX_G_NRML: i32 = 4; // sim/flightmodel/forces/g_nrml
pub const IDX_ALPHA: i32 = 5; // sim/flightmodel/position/alpha (deg)
pub const IDX_STALL_WARN: i32 = 6; // sim/cockpit2/annunciators/stall_warning (0/1)
pub const IDX_OVER_VNE: i32 = 7; // sim/flightmodel/failures/over_vne (0/1)
pub const IDX_GEAR_DEPLOY: i32 = 8; // sim/flightmodel2/gear/deploy_ratio[0]
pub const IDX_FLAP_RATIO: i32 = 9; // sim/flightmodel2/controls/flap_ratio
pub const IDX_ENGINE_N1: i32 = 10; // sim/flightmodel/engine/ENGN_N1_[0]
pub const IDX_IAS: i32 = 11; // sim/flightmodel/position/indicated_airspeed (kias)
pub const IDX_G_SIDE: i32 = 12; // sim/flightmodel/forces/g_side
pub const IDX_SPEEDBRAKE: i32 = 13; // sim/flightmodel2/controls/speedbrake_ratio
pub const IDX_SURFACE: i32 = 14; // sim/flightmodel/ground/surface_texture_type (enum)
pub const IDX_PARKING_BRAKE: i32 = 15; // sim/cockpit2/controls/parking_brake_ratio
pub const IDX_REVERSER: i32 = 16; // sim/flightmodel2/engines/thrust_reverser_deploy_ratio[0]
pub const IDX_AP_SERVOS: i32 = 17; // sim/cockpit2/autopilot/servos_on (0/1)
pub const IDX_OVER_VFE: i32 = 18; // sim/flightmodel/failures/over_vfe (0/1)
pub const IDX_OVER_VLE: i32 = 19; // sim/flightmodel/failures/over_vle (0/1)
pub const IDX_NOSE_ON_GROUND: i32 = 20; // sim/flightmodel2/gear/on_ground[0] (nose/tail wheel)
pub const IDX_IS_HELI: i32 = 21; // sim/aircraft2/metadata/is_helicopter (0/1)
pub const IDX_PROP_RPM: i32 = 22; // sim/cockpit2/engine/indicators/prop_speed_rpm[0]
pub const IDX_STALL_WARN_ALPHA: i32 = 23; // sim/aircraft/overflow/acf_stall_warn_alpha (deg)
pub const IDX_ELEVATION: i32 = 24; // sim/flightmodel/position/elevation (m MSL)

/// sim/flightmodel/engine/ENGN_running[0..N_ENGINES]
pub const IDX_ENGINE_RUNNING_BASE: i32 = 30;
pub const N_ENGINES: usize = 4;
/// sim/weather/aircraft/turbulence[0..13] (0-10 scale per wind layer)
pub const IDX_TURB_BASE: i32 = 40;
/// sim/weather/aircraft/wind_altitude_msl_m[0..13] (layer centre altitude)
pub const IDX_WIND_ALT_BASE: i32 = 60;
pub const N_WIND_LAYERS: usize = 13;

/// X-Plane surface_texture_type enum values (full list kept for reference;
/// paved types share the default arm in the mixer).
#[allow(dead_code)]
pub mod surface {
    pub const WATER: i32 = 1;
    pub const CONCRETE: i32 = 2;
    pub const ASPHALT: i32 = 3;
    pub const GRASS: i32 = 4;
    pub const DIRT: i32 = 5;
    pub const GRAVEL: i32 = 6;
    pub const LAKE: i32 = 7;
    pub const SNOW: i32 = 8;
    pub const SHOULDER: i32 = 9;
    pub const BLASTPAD: i32 = 10;
    pub const GROUND: i32 = 11;
}

/// Build the full (dataref path, index, requested Hz) subscription list.
pub fn subscriptions() -> Vec<(String, i32, i32)> {
    let mut v: Vec<(String, i32, i32)> = [
        ("sim/flightmodel/position/groundspeed", IDX_GROUNDSPEED, 30),
        ("sim/flightmodel/position/vh_ind_fpm", IDX_VH_IND_FPM, 30),
        ("sim/flightmodel/failures/onground_any", IDX_ONGROUND_ANY, 30),
        ("sim/flightmodel/forces/g_nrml", IDX_G_NRML, 30),
        ("sim/flightmodel/position/alpha", IDX_ALPHA, 20),
        ("sim/cockpit2/annunciators/stall_warning", IDX_STALL_WARN, 20),
        ("sim/flightmodel/failures/over_vne", IDX_OVER_VNE, 10),
        ("sim/flightmodel2/gear/deploy_ratio[0]", IDX_GEAR_DEPLOY, 10),
        ("sim/flightmodel2/controls/flap_ratio", IDX_FLAP_RATIO, 10),
        ("sim/flightmodel/engine/ENGN_N1_[0]", IDX_ENGINE_N1, 10),
        ("sim/flightmodel/position/indicated_airspeed", IDX_IAS, 20),
        ("sim/flightmodel/forces/g_side", IDX_G_SIDE, 30),
        ("sim/flightmodel2/controls/speedbrake_ratio", IDX_SPEEDBRAKE, 10),
        ("sim/flightmodel/ground/surface_texture_type", IDX_SURFACE, 5),
        ("sim/cockpit2/controls/parking_brake_ratio", IDX_PARKING_BRAKE, 5),
        ("sim/flightmodel2/engines/thrust_reverser_deploy_ratio[0]", IDX_REVERSER, 10),
        ("sim/cockpit2/autopilot/servos_on", IDX_AP_SERVOS, 10),
        ("sim/flightmodel/failures/over_vfe", IDX_OVER_VFE, 10),
        ("sim/flightmodel/failures/over_vle", IDX_OVER_VLE, 10),
        ("sim/flightmodel2/gear/on_ground[0]", IDX_NOSE_ON_GROUND, 30),
        ("sim/aircraft2/metadata/is_helicopter", IDX_IS_HELI, 1),
        ("sim/cockpit2/engine/indicators/prop_speed_rpm[0]", IDX_PROP_RPM, 10),
        ("sim/aircraft/overflow/acf_stall_warn_alpha", IDX_STALL_WARN_ALPHA, 1),
        ("sim/flightmodel/position/elevation", IDX_ELEVATION, 2),
    ]
    .iter()
    .map(|(d, i, f)| (d.to_string(), *i, *f))
    .collect();

    for e in 0..N_ENGINES {
        v.push((format!("sim/flightmodel/engine/ENGN_running[{e}]"), IDX_ENGINE_RUNNING_BASE + e as i32, 5));
    }
    for l in 0..N_WIND_LAYERS {
        v.push((format!("sim/weather/aircraft/turbulence[{l}]"), IDX_TURB_BASE + l as i32, 2));
        v.push((format!("sim/weather/aircraft/wind_altitude_msl_m[{l}]"), IDX_WIND_ALT_BASE + l as i32, 1));
    }
    v
}

/// A one-shot event the mixer should render as a decaying transient rather
/// than a continuous level (touchdown thump, gear/flap bump, etc).
/// `started_at` may be in the future, which is how multi-pulse patterns
/// (e.g. the autopilot-disconnect double tap) are scheduled.
#[derive(Debug, Clone, Copy)]
pub struct Transient {
    pub started_at: Instant,
    pub peak_amplitude: f32, // 0.0-1.0
    pub duration_ms: u32,
}

impl Transient {
    pub fn now(peak_amplitude: f32, duration_ms: u32) -> Self {
        Self { started_at: Instant::now(), peak_amplitude, duration_ms }
    }

    pub fn after(delay_ms: u64, peak_amplitude: f32, duration_ms: u32) -> Self {
        Self { started_at: Instant::now() + Duration::from_millis(delay_ms), peak_amplitude, duration_ms }
    }
}

/// Boolean edge detector that ignores the very first sample, so connecting
/// mid-flight (engines already running, AP already engaged) doesn't fire
/// a spurious start/disconnect event.
#[derive(Debug, Default, Clone, Copy)]
struct Edge {
    seen: bool,
    prev: bool,
}

impl Edge {
    /// Returns Some(true) on rising edge, Some(false) on falling edge.
    fn update(&mut self, now: bool) -> Option<bool> {
        let edge = if self.seen && now != self.prev { Some(now) } else { None };
        self.seen = true;
        self.prev = now;
        edge
    }
}

#[derive(Debug, Default)]
pub struct FlightState {
    pub groundspeed_mps: f32,
    pub vh_ind_fpm: f32,
    pub onground: bool,
    pub g_nrml: f32,
    pub g_side: f32,
    pub alpha_deg: f32,
    pub stall_warning: bool,
    pub over_vne: bool,
    pub over_vfe: bool,
    pub over_vle: bool,
    pub gear_deploy_ratio: f32,
    pub flap_ratio: f32,
    pub engine_n1: f32,
    pub ias_kts: f32,
    pub speedbrake_ratio: f32,
    pub surface_type: i32,
    pub parking_brake_ratio: f32,
    pub reverser_ratio: f32,
    pub is_helicopter: bool,
    pub prop_rpm: f32,
    pub elevation_m: f32,
    pub turbulence_layers: [Option<f32>; N_WIND_LAYERS],
    pub wind_layer_alt_m: [Option<f32>; N_WIND_LAYERS],

    // Derived / internal
    prev_onground: bool,
    prev_gear_deploy_ratio: f32,
    prev_flap_ratio: f32,
    engine_edges: [Edge; N_ENGINES],
    ap_edge: Edge,
    nose_edge: Edge,
    main_touchdown_at: Option<Instant>,
    /// Time of the most recent RREF value applied; the mixer goes silent
    /// when this goes stale (X-Plane paused/quit/crashed).
    pub last_update: Option<Instant>,

    pub touchdown: Option<Transient>,
    pub nose_touchdown: Option<Transient>,
    pub gear_bump: Option<Transient>,
    pub flap_bump: Option<Transient>,
    pub engine_pulse: Option<Transient>,
    pub ap_disconnect: [Option<Transient>; 2],

    /// Fallback stall AoA when neither the config override nor the
    /// aircraft's own acf_stall_warn_alpha is available.
    pub stall_alpha_deg: f32,
    /// Per-aircraft stall warning AoA from the .acf, 0 until received.
    pub acf_stall_warn_alpha: f32,
}

impl FlightState {
    pub fn new() -> Self {
        Self {
            stall_alpha_deg: 14.0,
            g_nrml: 1.0, // 1g at rest/level flight; the derived Default gives 0.0,
            // which would read as permanent freefall to the turbulence
            // proxy before the first real RREF packet arrives.
            surface_type: surface::ASPHALT,
            ..Default::default()
        }
    }

    /// Stall AoA to measure margin against: config override, else the
    /// aircraft's own stall-warning alpha, else the generic default.
    pub fn effective_stall_alpha(&self, override_deg: Option<f32>) -> f32 {
        if let Some(a) = override_deg.filter(|a| *a > 0.0) {
            return a;
        }
        if self.acf_stall_warn_alpha > 1.0 && self.acf_stall_warn_alpha < 45.0 {
            return self.acf_stall_warn_alpha;
        }
        self.stall_alpha_deg
    }

    /// Weather turbulence factor (0-10) at the aircraft's altitude,
    /// linearly interpolated between X-Plane 12's wind layers. None if the
    /// sim hasn't sent any (X-Plane 11, or not received yet).
    pub fn turbulence_at_aircraft(&self) -> Option<f32> {
        let mut layers: Vec<(f32, f32)> = (0..N_WIND_LAYERS)
            .filter_map(|i| self.turbulence_layers[i].map(|t| (self.wind_layer_alt_m[i].unwrap_or(0.0), t)))
            .collect();
        if layers.is_empty() {
            return None;
        }
        layers.sort_by(|a, b| a.0.total_cmp(&b.0));
        let alt = self.elevation_m;
        if alt <= layers[0].0 {
            return Some(layers[0].1);
        }
        for w in layers.windows(2) {
            let ((a0, t0), (a1, t1)) = (w[0], w[1]);
            if alt <= a1 {
                let span = (a1 - a0).max(1.0);
                return Some(t0 + (t1 - t0) * ((alt - a0) / span));
            }
        }
        Some(layers.last().unwrap().1)
    }

    /// Apply one (index, value) pair from an RREF packet, updating derived
    /// state (touchdown/gear/flap/engine/AP transients) as needed.
    pub fn apply(&mut self, index: i32, value: f32) {
        self.last_update = Some(Instant::now());
        match index {
            IDX_GROUNDSPEED => self.groundspeed_mps = value,
            IDX_VH_IND_FPM => self.vh_ind_fpm = value,
            IDX_ONGROUND_ANY => {
                let now_on = value >= 0.5;
                if now_on && !self.prev_onground {
                    self.on_touchdown();
                }
                self.prev_onground = now_on;
                self.onground = now_on;
            }
            IDX_G_NRML => self.g_nrml = value,
            IDX_G_SIDE => self.g_side = value,
            IDX_ALPHA => self.alpha_deg = value,
            IDX_STALL_WARN => self.stall_warning = value >= 0.5,
            IDX_OVER_VNE => self.over_vne = value >= 0.5,
            IDX_OVER_VFE => self.over_vfe = value >= 0.5,
            IDX_OVER_VLE => self.over_vle = value >= 0.5,
            IDX_GEAR_DEPLOY => {
                if (value - self.prev_gear_deploy_ratio).abs() > 0.02 {
                    self.gear_bump = Some(Transient::now(0.5, 250));
                }
                self.prev_gear_deploy_ratio = value;
                self.gear_deploy_ratio = value;
            }
            IDX_FLAP_RATIO => {
                if (value - self.prev_flap_ratio).abs() > 0.05 {
                    self.flap_bump = Some(Transient::now(0.3, 180));
                }
                self.prev_flap_ratio = value;
                self.flap_ratio = value;
            }
            IDX_ENGINE_N1 => self.engine_n1 = value,
            IDX_IAS => self.ias_kts = value,
            IDX_SPEEDBRAKE => self.speedbrake_ratio = value,
            IDX_SURFACE => self.surface_type = value.round() as i32,
            IDX_PARKING_BRAKE => self.parking_brake_ratio = value,
            IDX_REVERSER => self.reverser_ratio = value,
            IDX_AP_SERVOS => {
                // Falling edge = autopilot disconnect (intentional or not):
                // two short taps, like the disconnect warning.
                if self.ap_edge.update(value >= 0.5) == Some(false) {
                    self.ap_disconnect = [Some(Transient::now(0.7, 90)), Some(Transient::after(170, 0.7, 90))];
                }
            }
            IDX_NOSE_ON_GROUND => {
                if self.nose_edge.update(value >= 0.5) == Some(true) {
                    self.on_nose_touchdown();
                }
            }
            IDX_IS_HELI => self.is_helicopter = value >= 0.5,
            IDX_PROP_RPM => self.prop_rpm = value,
            IDX_STALL_WARN_ALPHA => self.acf_stall_warn_alpha = value,
            IDX_ELEVATION => self.elevation_m = value,
            i if (IDX_ENGINE_RUNNING_BASE..IDX_ENGINE_RUNNING_BASE + N_ENGINES as i32).contains(&i) => {
                let e = (i - IDX_ENGINE_RUNNING_BASE) as usize;
                match self.engine_edges[e].update(value >= 0.5) {
                    // Light-off: a solid shove. Shutdown: a softer shudder.
                    Some(true) => self.engine_pulse = Some(Transient::now(0.7, 450)),
                    Some(false) => self.engine_pulse = Some(Transient::now(0.4, 350)),
                    None => {}
                }
            }
            i if (IDX_TURB_BASE..IDX_TURB_BASE + N_WIND_LAYERS as i32).contains(&i) => {
                self.turbulence_layers[(i - IDX_TURB_BASE) as usize] = Some(value);
            }
            i if (IDX_WIND_ALT_BASE..IDX_WIND_ALT_BASE + N_WIND_LAYERS as i32).contains(&i) => {
                self.wind_layer_alt_m[(i - IDX_WIND_ALT_BASE) as usize] = Some(value);
            }
            _ => {}
        }
    }

    fn on_touchdown(&mut self) {
        // Harder landings (more negative fpm, or a g spike) => bigger thump.
        // -60fpm greaser -> ~0.15, -600fpm arrival -> ~1.0, clamped.
        let fpm_severity = (-self.vh_ind_fpm / 600.0).clamp(0.0, 1.0);
        let g_severity = ((self.g_nrml - 1.0) / 1.5).clamp(0.0, 1.0);
        let severity = fpm_severity.max(g_severity).max(0.12); // floor so it's never silent
        self.touchdown = Some(Transient::now(severity, 220));
        self.main_touchdown_at = Some(Instant::now());
    }

    /// Nose (or tail) wheel coming down after the mains: a lighter, sharper
    /// thump. Only counts if the mains touched down a moment earlier, so a
    /// three-point landing doesn't double up with the main thump.
    fn on_nose_touchdown(&mut self) {
        let Some(main_at) = self.main_touchdown_at else { return };
        let since = main_at.elapsed();
        if since >= Duration::from_millis(250) && since <= Duration::from_secs(20) {
            let speed_factor = (self.groundspeed_mps / 60.0).clamp(0.3, 1.0);
            self.nose_touchdown = Some(Transient::now(0.6 * speed_factor, 160));
        }
    }

    /// Test helper: pretend the main-gear touchdown happened `ms` ago.
    #[cfg(test)]
    pub fn backdate_main_touchdown(&mut self, ms: u64) {
        self.main_touchdown_at = Some(Instant::now() - Duration::from_millis(ms));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn engine_already_running_at_connect_does_not_pulse() {
        let mut fs = FlightState::new();
        fs.apply(IDX_ENGINE_RUNNING_BASE, 1.0);
        assert!(fs.engine_pulse.is_none());
        fs.apply(IDX_ENGINE_RUNNING_BASE, 0.0);
        assert!(fs.engine_pulse.is_some(), "shutdown after first sample should pulse");
    }

    #[test]
    fn ap_disconnect_schedules_double_tap() {
        let mut fs = FlightState::new();
        fs.apply(IDX_AP_SERVOS, 1.0);
        assert!(fs.ap_disconnect[0].is_none());
        fs.apply(IDX_AP_SERVOS, 0.0);
        let [a, b] = fs.ap_disconnect;
        let (a, b) = (a.unwrap(), b.unwrap());
        assert!(b.started_at > a.started_at, "second tap must be scheduled after the first");
    }

    #[test]
    fn nose_touchdown_only_after_mains() {
        let mut fs = FlightState::new();
        fs.apply(IDX_NOSE_ON_GROUND, 0.0);
        fs.apply(IDX_NOSE_ON_GROUND, 1.0);
        assert!(fs.nose_touchdown.is_none(), "no main touchdown yet -> no nose thump");

        let mut fs = FlightState::new();
        fs.apply(IDX_NOSE_ON_GROUND, 0.0);
        fs.apply(IDX_ONGROUND_ANY, 0.0);
        fs.apply(IDX_ONGROUND_ANY, 1.0);
        fs.backdate_main_touchdown(1500);
        fs.apply(IDX_GROUNDSPEED, 60.0);
        fs.apply(IDX_NOSE_ON_GROUND, 1.0);
        assert!(fs.nose_touchdown.is_some());
    }

    #[test]
    fn turbulence_interpolates_between_layers() {
        let mut fs = FlightState::new();
        assert_eq!(fs.turbulence_at_aircraft(), None);
        fs.apply(IDX_WIND_ALT_BASE, 1000.0);
        fs.apply(IDX_TURB_BASE, 0.0);
        fs.apply(IDX_WIND_ALT_BASE + 1, 3000.0);
        fs.apply(IDX_TURB_BASE + 1, 8.0);
        fs.apply(IDX_ELEVATION, 2000.0);
        assert!((fs.turbulence_at_aircraft().unwrap() - 4.0).abs() < 1e-3);
        fs.apply(IDX_ELEVATION, 500.0);
        assert_eq!(fs.turbulence_at_aircraft(), Some(0.0));
        fs.apply(IDX_ELEVATION, 9000.0);
        assert_eq!(fs.turbulence_at_aircraft(), Some(8.0));
    }

    #[test]
    fn stall_alpha_priority() {
        let mut fs = FlightState::new();
        assert_eq!(fs.effective_stall_alpha(None), 14.0);
        fs.apply(IDX_STALL_WARN_ALPHA, 11.5);
        assert_eq!(fs.effective_stall_alpha(None), 11.5);
        assert_eq!(fs.effective_stall_alpha(Some(16.0)), 16.0);
    }

    #[test]
    fn subscriptions_have_unique_indices() {
        let subs = subscriptions();
        let mut idx: Vec<i32> = subs.iter().map(|s| s.1).collect();
        idx.sort();
        idx.dedup();
        assert_eq!(idx.len(), subs.len());
        assert!(subs.iter().all(|(d, _, _)| d.len() < 400));
    }
}
