//! Dataref registry + the live flight state the effect mixer reads from.
//!
//! Each dataref we care about gets a fixed integer index (used as the RREF
//! subscription tag), so incoming (index, value) pairs can be routed with a
//! plain match instead of string comparisons on the hot path.

use std::time::Instant;

// --- Dataref index assignments -------------------------------------------
pub const IDX_GROUNDSPEED: i32 = 1; // sim/flightmodel/position/groundspeed (m/s)
pub const IDX_VH_IND_FPM: i32 = 2; // sim/flightmodel/position/vh_ind_fpm
pub const IDX_ONGROUND_ANY: i32 = 3; // sim/flightmodel/failures/onground_any (0/1)
pub const IDX_G_NRML: i32 = 4; // sim/flightmodel/forces/g_nrml
pub const IDX_ALPHA: i32 = 5; // sim/flightmodel/position/alpha (deg)
pub const IDX_STALL_WARN: i32 = 6; // sim/cockpit2/annunciators/stall_warning (0/1)
pub const IDX_OVER_VNE: i32 = 7; // sim/flightmodel/failures/over_vne (0/1)
pub const IDX_GEAR_DEPLOY: i32 = 8; // sim/flightmodel2/gear/deploy_ratio (avg-ish, single index for simplicity)
pub const IDX_FLAP_RATIO: i32 = 9; // sim/flightmodel2/controls/flap_ratio
pub const IDX_ENGINE_N1: i32 = 10; // sim/flightmodel/engine/ENGN_N1_0

/// (dataref path, index, requested frequency in Hz)
pub const SUBSCRIPTIONS: &[(&str, i32, i32)] = &[
    ("sim/flightmodel/position/groundspeed", IDX_GROUNDSPEED, 30),
    ("sim/flightmodel/position/vh_ind_fpm", IDX_VH_IND_FPM, 30),
    (
        "sim/flightmodel/failures/onground_any",
        IDX_ONGROUND_ANY,
        30,
    ),
    ("sim/flightmodel/forces/g_nrml", IDX_G_NRML, 30),
    ("sim/flightmodel/position/alpha", IDX_ALPHA, 20),
    (
        "sim/cockpit2/annunciators/stall_warning",
        IDX_STALL_WARN,
        20,
    ),
    ("sim/flightmodel/failures/over_vne", IDX_OVER_VNE, 10),
    ("sim/flightmodel2/gear/deploy_ratio", IDX_GEAR_DEPLOY, 10),
    ("sim/flightmodel2/controls/flap_ratio", IDX_FLAP_RATIO, 10),
    ("sim/flightmodel/engine/ENGN_N1_0", IDX_ENGINE_N1, 10),
];

/// A one-shot event the mixer should render as a decaying transient rather
/// than a continuous level (touchdown thump, gear/flap bump, etc).
#[derive(Debug, Clone, Copy)]
pub struct Transient {
    pub started_at: Instant,
    pub peak_amplitude: f32, // 0.0-1.0
    pub duration_ms: u32,
}

#[derive(Debug, Default)]
pub struct FlightState {
    pub groundspeed_mps: f32,
    pub vh_ind_fpm: f32,
    pub onground: bool,
    pub g_nrml: f32,
    pub alpha_deg: f32,
    pub stall_warning: bool,
    pub over_vne: bool,
    pub gear_deploy_ratio: f32,
    pub flap_ratio: f32,
    pub engine_n1: f32,

    // Derived / internal
    prev_onground: bool,
    prev_gear_deploy_ratio: f32,
    prev_flap_ratio: f32,
    pub touchdown: Option<Transient>,
    pub gear_bump: Option<Transient>,
    pub flap_bump: Option<Transient>,
    /// Estimated stall AoA; conservative generic default, override per-aircraft
    /// later via sim/aircraft/overflow/acf_stall_warn_alpha if desired.
    pub stall_alpha_deg: f32,
}

impl FlightState {
    pub fn new() -> Self {
        Self {
            stall_alpha_deg: 14.0,
            g_nrml: 1.0, // 1g at rest/level flight; the derived Default gives 0.0,
            // which would read as permanent freefall to the turbulence
            // proxy before the first real RREF packet arrives.
            ..Default::default()
        }
    }

    /// Apply one (index, value) pair from an RREF packet, updating derived
    /// state (touchdown/gear/flap transients) as needed.
    pub fn apply(&mut self, index: i32, value: f32) {
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
            IDX_ALPHA => self.alpha_deg = value,
            IDX_STALL_WARN => self.stall_warning = value >= 0.5,
            IDX_OVER_VNE => self.over_vne = value >= 0.5,
            IDX_GEAR_DEPLOY => {
                if (value - self.prev_gear_deploy_ratio).abs() > 0.02 {
                    self.gear_bump = Some(Transient {
                        started_at: Instant::now(),
                        peak_amplitude: 0.5,
                        duration_ms: 250,
                    });
                }
                self.prev_gear_deploy_ratio = value;
                self.gear_deploy_ratio = value;
            }
            IDX_FLAP_RATIO => {
                if (value - self.prev_flap_ratio).abs() > 0.05 {
                    self.flap_bump = Some(Transient {
                        started_at: Instant::now(),
                        peak_amplitude: 0.3,
                        duration_ms: 180,
                    });
                }
                self.prev_flap_ratio = value;
                self.flap_ratio = value;
            }
            IDX_ENGINE_N1 => self.engine_n1 = value,
            _ => {}
        }
    }

    fn on_touchdown(&mut self) {
        // Harder landings (more negative fpm, or a g spike) => bigger thump.
        // -60fpm greaser -> ~0.15, -600fpm arrival -> ~1.0, clamped.
        let fpm_severity = (-self.vh_ind_fpm / 600.0).clamp(0.0, 1.0);
        let g_severity = ((self.g_nrml - 1.0) / 1.5).clamp(0.0, 1.0);
        let severity = fpm_severity.max(g_severity).max(0.12); // floor so it's never silent
        self.touchdown = Some(Transient {
            started_at: Instant::now(),
            peak_amplitude: severity,
            duration_ms: 220,
        });
    }
}
