//! Turns `FlightState` into a per-tick `(left, right)` motor amplitude pair,
//! plus lightbar color as a simple AoA-margin indicator. Continuous effects
//! (ground rumble, stall buffet, turbulence-by-proxy, engine idle) are added
//! together and clamped; one-shot transients (touchdown, gear/flap bumps)
//! render as a decaying envelope layered on top.

use crate::config::Gains;
use crate::state::{FlightState, Transient};
use std::time::Instant;

/// Below this normalized level (post-gain, pre-scaling to 0-255), we force
/// output to exactly zero. Real DualSense haptics in games are silent at
/// baseline and only speak up for actual events -- a small constant hum
/// (idle engine noise, natural g_nrml jitter in smooth air) reads as "broken
/// controller," not "immersive." This deadzone keeps quiet flight quiet.
const OUTPUT_DEADZONE: f32 = 0.10;

pub struct EffectMixer {
    gains: Gains,
}

fn envelope(t: &Transient, now: Instant) -> f32 {
    let elapsed = now.saturating_duration_since(t.started_at).as_millis() as f32;
    let dur = t.duration_ms as f32;
    if elapsed >= dur {
        0.0
    } else {
        // Simple linear decay; good enough for short thumps/bumps.
        t.peak_amplitude * (1.0 - elapsed / dur)
    }
}

impl EffectMixer {
    pub fn new(gains: Gains) -> Self {
        Self { gains }
    }

    /// Returns (left_motor, right_motor) as 0-255 HID rumble magnitudes,
    /// plus an RGB lightbar color derived from AoA margin to stall.
    ///
    /// Design intent: continuous effects (ground rumble, buffet, turbulence)
    /// each have their own deadzone/threshold so normal cruise flight and
    /// idle engines produce *zero* output, not a faint constant hum. Only
    /// genuine events -- fast ground roll, real proximity to stall, actual
    /// turbulence, transients -- should move the motors at all.
    pub fn compute(&self, state: &FlightState) -> (u8, u8, (u8, u8, u8)) {
        let now = Instant::now();
        let mut level: f32 = 0.0;

        // 1. Ground rumble: only once actually rolling with some speed --
        //    taxiing at a crawl or sitting at the gate should be silent.
        if state.onground {
            const TAXI_FLOOR_MPS: f32 = 3.0; // ~6 kt; below this, no rumble at all
            if state.groundspeed_mps > TAXI_FLOOR_MPS {
                let speed_norm = ((state.groundspeed_mps - TAXI_FLOOR_MPS) / 55.0).clamp(0.0, 1.0);
                level += speed_norm * self.gains.ground_rumble;
            }
        }

        // 2. Stall buffet: stay at zero until genuinely close to stall AoA
        //    (was 4deg margin, which many aircraft sit inside of at normal
        //    cruise/climb AoA -- tightened to 2deg, and squared so it ramps
        //    in gently rather than snapping to audible immediately).
        let alpha_margin = state.stall_alpha_deg - state.alpha_deg;
        const STALL_MARGIN_THRESHOLD_DEG: f32 = 2.0;
        if alpha_margin < STALL_MARGIN_THRESHOLD_DEG {
            let buffet_norm = ((STALL_MARGIN_THRESHOLD_DEG - alpha_margin) / STALL_MARGIN_THRESHOLD_DEG).clamp(0.0, 1.0);
            level += buffet_norm.powi(2) * self.gains.stall_buffet;
        }
        if state.stall_warning {
            level += self.gains.stall_buffet;
        }

        // 3. Turbulence proxy: g_nrml jitters by a few hundredths even in
        //    smooth air just from sim noise/control inputs -- subtract that
        //    baseline out before it counts as "turbulence."
        const TURB_DEADBAND: f32 = 0.06;
        let turb_raw = (state.g_nrml - 1.0).abs();
        let turb_norm = (turb_raw - TURB_DEADBAND).max(0.0).clamp(0.0, 1.0);
        level += turb_norm * self.gains.turbulence;

        // 4. Overspeed/Mach buffet: discrete high-amplitude buzz, genuinely
        //    event-driven already (over_vne is a hard boolean), left as-is.
        if state.over_vne {
            level += self.gains.overspeed;
        }

        // 5. Engine idle rumble is OFF by default (see config.rs) -- this is
        //    exactly the "vibrating just because engines are on" complaint.
        //    Left in as an opt-in for people who do want a faint idle hum;
        //    gated behind its own threshold so low N1 doesn't count.
        if self.gains.engine_rumble > 0.0 {
            const ENGINE_N1_FLOOR: f32 = 60.0; // percent; below this, silent
            let n1 = state.engine_n1;
            if n1 > ENGINE_N1_FLOOR {
                let n1_norm = ((n1 - ENGINE_N1_FLOOR) / (100.0 - ENGINE_N1_FLOOR)).clamp(0.0, 1.0);
                level += n1_norm * self.gains.engine_rumble;
            }
        }

        // 6. One-shot transients layered on top via decaying envelopes --
        //    these are the "real haptic feedback" moments (touchdown,
        //    gear/flap) and are untouched by the deadzone logic above.
        let mut transient_level: f32 = 0.0;
        if let Some(t) = &state.touchdown {
            transient_level += envelope(t, now) * self.gains.touchdown;
        }
        if let Some(t) = &state.gear_bump {
            transient_level += envelope(t, now) * self.gains.gear_flap;
        }
        if let Some(t) = &state.flap_bump {
            transient_level += envelope(t, now) * self.gains.gear_flap;
        }
        level += transient_level;

        // Continuous effects respect the deadzone; a transient in-flight
        // (even a small one) always gets through so short events don't get
        // eaten by the same threshold that keeps cruise silent.
        let level = if transient_level > 0.0 {
            level.clamp(0.0, 1.0)
        } else if level < OUTPUT_DEADZONE {
            0.0
        } else {
            level.clamp(0.0, 1.0)
        };

        let magnitude = (level * 255.0).round() as u8;

        // Split identically across both actuators for now; the DualSense's
        // two VCMs have slightly different frequency response (one leans
        // low, one leans high) so a future refinement is biasing thumps to
        // one side and rumble/buffet to the other.
        let (left, right) = (magnitude, magnitude);

        // Lightbar: green with margin, sliding to red as alpha margin closes,
        // matching the "AoA indicator" idea from HorizonHaptics-style tools.
        let margin_norm = (alpha_margin / 8.0).clamp(0.0, 1.0);
        let r = ((1.0 - margin_norm) * 255.0) as u8;
        let g = (margin_norm * 255.0) as u8;
        let lightbar = (r, g, 0);

        (left, right, lightbar)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Gains;
    use crate::state::FlightState;

    fn default_mixer() -> EffectMixer {
        EffectMixer::new(Gains::default())
    }

    #[test]
    fn cruise_flight_is_silent() {
        // Airborne, steady, mid-AoA, engines running -- should be dead quiet.
        let mut fs = FlightState::new();
        fs.apply(crate::state::IDX_ONGROUND_ANY, 0.0);
        fs.apply(crate::state::IDX_ALPHA, 3.0); // comfortably below stall margin threshold
        fs.apply(crate::state::IDX_G_NRML, 1.01); // tiny natural jitter, within deadband
        fs.apply(crate::state::IDX_ENGINE_N1, 85.0); // engines clearly running
        let (l, r, _) = default_mixer().compute(&fs);
        assert_eq!((l, r), (0, 0), "cruise flight with engines on should produce zero rumble");
    }

    #[test]
    fn slow_taxi_is_silent_but_fast_taxi_is_not() {
        let mut fs_slow = FlightState::new();
        // Establish an airborne baseline first so the onground=1.0 below is a
        // genuine rising edge we then let decay, not the *first* transition
        // from the type's default (which the mixer correctly treats as a
        // touchdown -- see the code comment on prev_onground).
        fs_slow.apply(crate::state::IDX_ONGROUND_ANY, 0.0);
        fs_slow.apply(crate::state::IDX_ONGROUND_ANY, 1.0);
        fs_slow.touchdown = None; // discard the transient; we're testing steady-state taxi
        fs_slow.apply(crate::state::IDX_GROUNDSPEED, 1.5); // ~3kt, below taxi floor
        let (l, r, _) = default_mixer().compute(&fs_slow);
        assert_eq!((l, r), (0, 0), "slow taxi should be silent");

        let mut fs_fast = FlightState::new();
        fs_fast.apply(crate::state::IDX_ONGROUND_ANY, 0.0);
        fs_fast.apply(crate::state::IDX_ONGROUND_ANY, 1.0);
        fs_fast.touchdown = None;
        fs_fast.apply(crate::state::IDX_GROUNDSPEED, 40.0); // ~78kt takeoff roll
        let (l, r, _) = default_mixer().compute(&fs_fast);
        assert!(l > 0 && r > 0, "fast ground roll should produce rumble");
    }

    #[test]
    fn touchdown_transient_gets_through_deadzone() {
        let mut fs = FlightState::new();
        // Simulate a touchdown: vertical speed set, then onground flips 0->1.
        fs.apply(crate::state::IDX_VH_IND_FPM, -300.0);
        fs.apply(crate::state::IDX_G_NRML, 1.3);
        fs.apply(crate::state::IDX_ONGROUND_ANY, 0.0);
        fs.apply(crate::state::IDX_ONGROUND_ANY, 1.0); // rising edge -> latches touchdown
        let (l, r, _) = default_mixer().compute(&fs);
        assert!(l > 0 && r > 0, "a fresh touchdown transient must not be swallowed by the deadzone");
    }

    #[test]
    fn engine_rumble_disabled_by_default_even_at_high_n1() {
        let mut fs = FlightState::new();
        fs.apply(crate::state::IDX_ONGROUND_ANY, 0.0);
        fs.apply(crate::state::IDX_ALPHA, 3.0);
        fs.apply(crate::state::IDX_ENGINE_N1, 100.0); // full power
        let (l, r, _) = default_mixer().compute(&fs);
        assert_eq!((l, r), (0, 0), "engine_rumble gain defaults to 0.0 and should stay silent");
    }
}
