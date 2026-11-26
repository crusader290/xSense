//! Turns `FlightState` into per-tick controller output: two rumble motor
//! amplitudes, a lightbar colour (AoA-margin indicator) and adaptive trigger
//! effects for R2 (throttle) and L2 (brakes).
//!
//! Motor routing: the DualSense's left actuator is the heavy, low-frequency
//! one and the right is light and high-frequency. Heavy things (ground roll,
//! touchdown, engine light-off, gear) go left; buzzes and buffets (stall,
//! overspeed, speedbrakes, AP disconnect) go right; turbulence uses both.
//! Routing them apart keeps simultaneous effects distinguishable.
//!
//! Continuous effects each have their own threshold and the per-channel sum
//! has a deadzone, so cruise flight stays silent. One-shot transients render
//! as decaying envelopes layered on top and always get through.

use crate::config::{Gains, TriggerConfig};
use crate::hid::TriggerEffect;
use crate::state::{surface, FlightState, Transient};
use std::f32::consts::TAU;
use std::time::{Duration, Instant};

/// Below this normalized level per motor (with no transient playing), output
/// is forced to exactly zero. A small constant hum reads as "broken
/// controller," not "immersive."
const OUTPUT_DEADZONE: f32 = 0.10;
/// No RREF data for this long => X-Plane paused/quit; go silent.
const STALE_AFTER: Duration = Duration::from_millis(1500);
/// ~6 kt; below this, no ground rumble at all.
const TAXI_FLOOR_MPS: f32 = 3.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MixOutput {
    pub left: u8,
    pub right: u8,
    pub lightbar: (u8, u8, u8),
    /// `None` = leave the trigger alone (feature disabled in config).
    pub right_trigger: Option<TriggerEffect>,
    pub left_trigger: Option<TriggerEffect>,
}

pub struct EffectMixer {
    gains: Gains,
    triggers: TriggerConfig,
    stall_alpha_override: Option<f32>,
    started: Instant,
    last_tick: Option<Instant>,
    // Turbulence high-pass: low-passed g values, so sustained g (a steady
    // turn, a pull-up) doesn't count, only the jolts on top of it.
    g_nrml_lp: f32,
    g_side_lp: f32,
    lp_init: bool,
    rotor_phase: f32,
    rough_hold: f32,
    rough_timer: f32,
    rng: u32,
}

fn envelope(t: &Transient, now: Instant) -> f32 {
    // Transients may be scheduled in the future (multi-pulse patterns).
    let Some(elapsed) = now.checked_duration_since(t.started_at) else {
        return 0.0;
    };
    let elapsed = elapsed.as_millis() as f32;
    let dur = t.duration_ms as f32;
    if elapsed >= dur {
        0.0
    } else {
        t.peak_amplitude * (1.0 - elapsed / dur)
    }
}

fn env(t: &Option<Transient>, now: Instant) -> f32 {
    t.as_ref().map_or(0.0, |t| envelope(t, now))
}

fn to_u8(level: f32) -> u8 {
    (level.clamp(0.0, 1.0) * 255.0).round() as u8
}

/// Apply the deadzone to one motor channel's continuous level, letting any
/// active transient through untouched.
fn finish_channel(continuous: f32, transient: f32) -> u8 {
    if transient > 0.0 {
        to_u8(continuous + transient)
    } else if continuous < OUTPUT_DEADZONE {
        0
    } else {
        to_u8(continuous)
    }
}

impl EffectMixer {
    pub fn new(gains: Gains, triggers: TriggerConfig, stall_alpha_override: Option<f32>) -> Self {
        Self {
            gains,
            triggers,
            stall_alpha_override,
            started: Instant::now(),
            last_tick: None,
            g_nrml_lp: 1.0,
            g_side_lp: 0.0,
            lp_init: false,
            rotor_phase: 0.0,
            rough_hold: 0.0,
            rough_timer: 0.0,
            rng: 0x9E37_79B9,
        }
    }

    pub fn compute(&mut self, state: &FlightState) -> MixOutput {
        self.compute_at(state, Instant::now())
    }

    fn rand01(&mut self) -> f32 {
        // xorshift32: cheap, deterministic, plenty for bump texture.
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        (x >> 8) as f32 / (1u32 << 24) as f32
    }

    pub fn compute_at(&mut self, state: &FlightState, now: Instant) -> MixOutput {
        let dt = self
            .last_tick
            .map(|t| now.saturating_duration_since(t).as_secs_f32())
            .unwrap_or(1.0 / 60.0)
            .clamp(0.0, 0.1);
        self.last_tick = Some(now);

        let stall_alpha = state.effective_stall_alpha(self.stall_alpha_override);
        let alpha_margin = stall_alpha - state.alpha_deg;

        let stale = state.last_update.is_none_or(|t| now.saturating_duration_since(t) > STALE_AFTER);
        if stale {
            let off = self.triggers.enabled.then_some(TriggerEffect::Off);
            self.lp_init = false;
            return MixOutput { left: 0, right: 0, lightbar: (0, 0, 32), right_trigger: off, left_trigger: off };
        }

        let g = self.gains.clone();
        let t = now.saturating_duration_since(self.started).as_secs_f32();
        let airborne = !state.onground;
        let ias = state.ias_kts.max(0.0);
        let (mut l, mut r) = (0.0f32, 0.0f32);

        // 1. Ground roll, textured by surface type.
        if state.onground && state.groundspeed_mps > TAXI_FLOOR_MPS {
            let speed_norm = ((state.groundspeed_mps - TAXI_FLOOR_MPS) / 55.0).clamp(0.0, 1.0);
            // Rough surfaces get bumpy at much lower speeds than pavement.
            let rough_norm = ((state.groundspeed_mps - TAXI_FLOOR_MPS) / 20.0).clamp(0.0, 1.0);
            match state.surface_type {
                surface::GRASS | surface::DIRT | surface::GRAVEL | surface::SHOULDER | surface::GROUND => {
                    l += speed_norm * g.ground_rumble;
                    // Irregular sample-and-hold bumps rather than a smooth hum.
                    self.rough_timer -= dt;
                    if self.rough_timer <= 0.0 {
                        self.rough_hold = self.rand01();
                        self.rough_timer = 0.05 + 0.06 * self.rand01();
                    }
                    l += rough_norm * g.surface_rough * (0.3 + 0.7 * self.rough_hold);
                    r += rough_norm * g.surface_rough * 0.7 * self.rough_hold;
                }
                surface::WATER | surface::LAKE => {
                    // Float/hull slap: slower, rolling pulses.
                    let slap = 0.5 + 0.5 * (TAU * (1.5 + 2.5 * rough_norm) * t).sin();
                    l += rough_norm * g.surface_rough * slap;
                }
                surface::SNOW => l += speed_norm * g.ground_rumble * 0.6,
                // Concrete / asphalt / blastpad / unknown: smooth, steady.
                _ => l += speed_norm * g.ground_rumble,
            }
        }

        // 2. Stall buffet + warning (right): ramps in only within 2 degrees.
        const STALL_MARGIN_THRESHOLD_DEG: f32 = 2.0;
        if alpha_margin < STALL_MARGIN_THRESHOLD_DEG && airborne {
            let buffet_norm = ((STALL_MARGIN_THRESHOLD_DEG - alpha_margin) / STALL_MARGIN_THRESHOLD_DEG).clamp(0.0, 1.0);
            r += buffet_norm.powi(2) * g.stall_buffet;
        }
        if state.stall_warning {
            r += g.stall_buffet;
        }

        // 3. Turbulence (both motors): jolts = deviation of g_nrml/g_side from
        //    their own recent average (high-pass), so sustained manoeuvring g
        //    doesn't register. Scaled by X-Plane 12's weather turbulence at the
        //    aircraft's altitude when available; plain proxy otherwise.
        let tau = 1.5;
        if !self.lp_init {
            self.g_nrml_lp = state.g_nrml;
            self.g_side_lp = state.g_side;
            self.lp_init = true;
        } else {
            let a = dt / (tau + dt);
            self.g_nrml_lp += (state.g_nrml - self.g_nrml_lp) * a;
            self.g_side_lp += (state.g_side - self.g_side_lp) * a;
        }
        if airborne {
            const TURB_DEADBAND: f32 = 0.04;
            let jn = state.g_nrml - self.g_nrml_lp;
            let js = (state.g_side - self.g_side_lp) * 1.5; // lateral jolts feel stronger
            let jolt = (jn * jn + js * js).sqrt();
            let jolt_norm = ((jolt - TURB_DEADBAND) / 0.4).clamp(0.0, 1.0);
            // Weather factor: calm air keeps a floor so wake encounters and
            // real bumps still come through; heavy turbulence is full scale.
            let weather = state.turbulence_at_aircraft().map_or(1.0, |turb| (0.35 + turb / 4.0).clamp(0.35, 1.0));
            let turb = jolt_norm * weather * g.turbulence;
            l += turb;
            r += turb * 0.7;
        }

        // 4. Overspeed: Vne steady buzz; flap/gear limit speeds pulse at 4 Hz
        //    so you can tell them apart.
        if state.over_vne {
            r += g.overspeed;
        }
        if (state.over_vfe || state.over_vle) && (t * 4.0).fract() < 0.5 {
            r += g.config_overspeed;
        }

        // 5. Speedbrake buffet (right): deployment x dynamic pressure.
        if state.speedbrake_ratio > 0.01 {
            let q = (ias / 250.0).powi(2).clamp(0.0, 1.0);
            r += state.speedbrake_ratio.clamp(0.0, 1.0) * q * g.speedbrake_buffet;
        }

        // 6. Gear-down airframe rumble (left), airborne, scales with airspeed.
        if airborne && state.gear_deploy_ratio > 0.01 {
            let q = (ias / 200.0).powi(2).clamp(0.0, 1.0);
            l += state.gear_deploy_ratio.clamp(0.0, 1.0) * q * g.gear_buffet;
        }

        // 7. Engine hum (opt-in), gated to N1 > 60%.
        if g.engine_rumble > 0.0 {
            const ENGINE_N1_FLOOR: f32 = 60.0;
            if state.engine_n1 > ENGINE_N1_FLOOR {
                let n1_norm = ((state.engine_n1 - ENGINE_N1_FLOOR) / (100.0 - ENGINE_N1_FLOOR)).clamp(0.0, 1.0);
                l += n1_norm * g.engine_rumble;
            }
        }

        // 8. Rotor beat (helicopters, left): pulses at the rotor's once-per-
        //    rev rate when that's slow enough to feel. Prop hum (fixed wing,
        //    right) is opt-in and steady -- props spin too fast to pulse.
        if state.is_helicopter && g.rotor_beat > 0.0 && state.prop_rpm > 30.0 {
            let f = state.prop_rpm / 60.0;
            self.rotor_phase = (self.rotor_phase + TAU * f * dt) % TAU;
            let pulse = if f < 25.0 { 0.5 + 0.5 * self.rotor_phase.sin() } else { 1.0 };
            let spool = (state.prop_rpm / 200.0).clamp(0.0, 1.0);
            l += g.rotor_beat * spool * pulse;
        } else if !state.is_helicopter && g.prop_beat > 0.0 && state.prop_rpm > 300.0 {
            r += g.prop_beat * (state.prop_rpm / 2700.0).clamp(0.0, 1.0);
        }

        // 9. Transients.
        let td = env(&state.touchdown, now) * g.touchdown;
        let nose = env(&state.nose_touchdown, now) * g.nose_touchdown;
        let eng = env(&state.engine_pulse, now) * g.engine_start;
        let gear = env(&state.gear_bump, now) * g.gear_flap;
        let flap = env(&state.flap_bump, now) * g.gear_flap;
        let ap = (env(&state.ap_disconnect[0], now) + env(&state.ap_disconnect[1], now)) * g.ap_disconnect;
        let lt = td + 0.5 * nose + eng + gear;
        let rt = 0.5 * td + nose + flap + ap;

        let left = finish_channel(l, lt);
        let right = finish_channel(r, rt);

        // Lightbar: green with margin, sliding to red as alpha margin closes.
        let margin_norm = (alpha_margin / 8.0).clamp(0.0, 1.0);
        let lightbar = (((1.0 - margin_norm) * 255.0) as u8, (margin_norm * 255.0) as u8, 0);

        let (right_trigger, left_trigger) = self.trigger_effects(state);
        MixOutput { left, right, lightbar, right_trigger, left_trigger }
    }

    fn trigger_effects(&self, state: &FlightState) -> (Option<TriggerEffect>, Option<TriggerEffect>) {
        let c = &self.triggers;
        if !c.enabled {
            return (None, None);
        }
        let manage_r = c.throttle || c.reverse_buzz || c.stall_shaker;
        let manage_l = c.brakes || c.stall_shaker;

        // Stick shaker overrides everything else on both triggers.
        if c.stall_shaker && state.stall_warning {
            let shake = TriggerEffect::Vibration { start: 0, amplitude: 8, frequency: c.stall_shaker_hz.max(1) };
            return (manage_r.then_some(shake), manage_l.then_some(shake));
        }

        let right = if c.reverse_buzz && state.reverser_ratio > 0.1 {
            // Reverse thrust: R2 buzzes so you know you're in reverse.
            TriggerEffect::Vibration { start: 0, amplitude: 3, frequency: 30 }
        } else if c.throttle {
            let zone = c.throttle_detent_zone.clamp(1, 9) as usize;
            let mut strengths = [1u8; 10];
            for s in strengths.iter_mut().skip(zone) {
                *s = c.throttle_detent_strength.clamp(1, 8);
            }
            TriggerEffect::MultiFeedback { strengths }
        } else {
            TriggerEffect::Off
        };

        let left = if c.brakes && state.onground {
            if state.parking_brake_ratio >= 0.5 {
                TriggerEffect::Feedback { start: 0, strength: 8 }
            } else {
                let speed_norm = (state.groundspeed_mps / 40.0).clamp(0.0, 1.0);
                TriggerEffect::Feedback { start: 0, strength: 2 + (speed_norm * 4.0).round() as u8 }
            }
        } else {
            TriggerEffect::Off
        };

        (manage_r.then_some(right), manage_l.then_some(left))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Gains, TriggerConfig};
    use crate::state::*;

    fn default_mixer() -> EffectMixer {
        EffectMixer::new(Gains::default(), TriggerConfig::default(), None)
    }

    fn on_ground_rolling(gs: f32) -> FlightState {
        let mut fs = FlightState::new();
        fs.apply(IDX_ONGROUND_ANY, 0.0);
        fs.apply(IDX_ONGROUND_ANY, 1.0);
        fs.touchdown = None; // steady-state ground roll, not the landing
        fs.apply(IDX_GROUNDSPEED, gs);
        fs
    }

    #[test]
    fn cruise_flight_is_silent() {
        let mut fs = FlightState::new();
        fs.apply(IDX_ONGROUND_ANY, 0.0);
        fs.apply(IDX_ALPHA, 3.0);
        fs.apply(IDX_G_NRML, 1.01);
        fs.apply(IDX_ENGINE_N1, 85.0);
        fs.apply(IDX_IAS, 250.0);
        let o = default_mixer().compute(&fs);
        assert_eq!((o.left, o.right), (0, 0), "cruise flight with engines on should produce zero rumble");
    }

    #[test]
    fn slow_taxi_is_silent_but_fast_taxi_rumbles_left() {
        let o = default_mixer().compute(&on_ground_rolling(1.5));
        assert_eq!((o.left, o.right), (0, 0), "slow taxi should be silent");

        let o = default_mixer().compute(&on_ground_rolling(40.0));
        assert!(o.left > 0, "fast ground roll should rumble the heavy motor");
        assert_eq!(o.right, 0, "smooth pavement shouldn't buzz the light motor");
    }

    #[test]
    fn grass_is_rougher_than_asphalt_at_taxi_speed() {
        // Grass texture is random, so sample a second of ticks.
        let run = |surface_type: i32| {
            let mut fs = on_ground_rolling(20.0);
            fs.apply(IDX_SURFACE, surface_type as f32);
            let mut m = default_mixer();
            let t0 = Instant::now();
            (0..60u64)
                .map(|i| {
                    let now = t0 + Duration::from_millis(i * 16);
                    fs.last_update = Some(now);
                    m.compute_at(&fs, now)
                })
                .collect::<Vec<_>>()
        };
        let asphalt = run(surface::ASPHALT);
        let grass = run(surface::GRASS);
        let max_l = |v: &[MixOutput]| v.iter().map(|o| o.left).max().unwrap();
        assert!(max_l(&grass) > max_l(&asphalt));
        assert!(asphalt.iter().all(|o| o.right == 0), "pavement is smooth");
        assert!(grass.iter().any(|o| o.right > 0), "grass adds high-frequency texture");
        let distinct: std::collections::HashSet<u8> = grass.iter().map(|o| o.left).collect();
        assert!(distinct.len() > 3, "grass rumble should be irregular, not a steady hum");
    }

    #[test]
    fn touchdown_transient_gets_through_deadzone() {
        let mut fs = FlightState::new();
        fs.apply(IDX_VH_IND_FPM, -300.0);
        fs.apply(IDX_G_NRML, 1.3);
        fs.apply(IDX_ONGROUND_ANY, 0.0);
        fs.apply(IDX_ONGROUND_ANY, 1.0);
        let o = default_mixer().compute(&fs);
        assert!(o.left > 0 && o.right > 0, "a fresh touchdown transient must not be swallowed by the deadzone");
        assert!(o.left > o.right, "touchdown thump is weighted to the heavy motor");
    }

    #[test]
    fn engine_rumble_disabled_by_default_even_at_high_n1() {
        let mut fs = FlightState::new();
        fs.apply(IDX_ONGROUND_ANY, 0.0);
        fs.apply(IDX_ALPHA, 3.0);
        fs.apply(IDX_ENGINE_N1, 100.0);
        let o = default_mixer().compute(&fs);
        assert_eq!((o.left, o.right), (0, 0));
    }

    #[test]
    fn stall_warning_buzzes_right_and_shakes_triggers() {
        let mut fs = FlightState::new();
        fs.apply(IDX_ONGROUND_ANY, 0.0);
        fs.apply(IDX_STALL_WARN, 1.0);
        let o = default_mixer().compute(&fs);
        assert!(o.right > 0);
        assert_eq!(o.left, 0, "stall buffet is routed to the light motor");
        assert!(matches!(o.right_trigger, Some(TriggerEffect::Vibration { .. })));
        assert!(matches!(o.left_trigger, Some(TriggerEffect::Vibration { .. })));
    }

    #[test]
    fn speedbrake_buffet_scales_with_airspeed() {
        let mut slow = FlightState::new();
        slow.apply(IDX_ONGROUND_ANY, 0.0);
        slow.apply(IDX_SPEEDBRAKE, 1.0);
        slow.apply(IDX_IAS, 80.0);
        let mut fast = FlightState::new();
        fast.apply(IDX_ONGROUND_ANY, 0.0);
        fast.apply(IDX_SPEEDBRAKE, 1.0);
        fast.apply(IDX_IAS, 280.0);
        let s = default_mixer().compute(&slow);
        let f = default_mixer().compute(&fast);
        assert_eq!(s.right, 0, "speedbrakes at 80 kt are below the deadzone");
        assert!(f.right > 0);
    }

    #[test]
    fn sustained_turn_is_not_turbulence_but_a_jolt_is() {
        let mut fs = FlightState::new();
        fs.apply(IDX_ONGROUND_ANY, 0.0);
        fs.apply(IDX_ALPHA, 3.0);
        fs.apply(IDX_G_NRML, 2.0); // steady 60-degree bank
        let mut m = default_mixer();
        let t0 = Instant::now();
        let mut o = m.compute_at(&fs, t0);
        for i in 1..=600 {
            fs.last_update = Some(t0 + Duration::from_millis(i * 16));
            o = m.compute_at(&fs, t0 + Duration::from_millis(i * 16));
        }
        assert_eq!((o.left, o.right), (0, 0), "after ~10s of steady 2g the high-pass should settle");

        fs.apply(IDX_G_NRML, 2.5); // sudden bump
        fs.last_update = Some(t0 + Duration::from_millis(601 * 16));
        let o = m.compute_at(&fs, t0 + Duration::from_millis(601 * 16));
        assert!(o.left > 0 && o.right > 0);
    }

    #[test]
    fn calm_weather_attenuates_turbulence() {
        let mk = |turb: Option<f32>| {
            let mut fs = FlightState::new();
            fs.apply(IDX_ONGROUND_ANY, 0.0);
            fs.apply(IDX_ALPHA, 3.0);
            if let Some(t) = turb {
                fs.apply(IDX_TURB_BASE, t);
            }
            let mut m = default_mixer();
            let t0 = Instant::now();
            m.compute_at(&fs, t0);
            fs.apply(IDX_G_NRML, 1.35);
            m.compute_at(&fs, t0 + Duration::from_millis(16)).left
        };
        let calm = mk(Some(0.0));
        let rough = mk(Some(6.0));
        let unknown = mk(None);
        assert!(rough > calm);
        assert_eq!(rough, unknown, "no weather data -> plain proxy at full scale");
    }

    #[test]
    fn ap_disconnect_double_tap_on_right() {
        let mut fs = FlightState::new();
        fs.apply(IDX_ONGROUND_ANY, 0.0);
        fs.apply(IDX_AP_SERVOS, 1.0);
        fs.apply(IDX_AP_SERVOS, 0.0);
        let mut m = default_mixer();
        let t0 = fs.ap_disconnect[0].unwrap().started_at;
        assert!(m.compute_at(&fs, t0 + Duration::from_millis(10)).right > 0, "first tap");
        assert_eq!(m.compute_at(&fs, t0 + Duration::from_millis(120)).right, 0, "gap");
        assert!(m.compute_at(&fs, t0 + Duration::from_millis(185)).right > 0, "second tap");
    }

    #[test]
    fn flap_overspeed_pulses() {
        let mut fs = FlightState::new();
        fs.apply(IDX_ONGROUND_ANY, 0.0);
        fs.apply(IDX_OVER_VFE, 1.0);
        let mut m = default_mixer();
        let base = m.started;
        let on = m.compute_at(&fs, base + Duration::from_millis(1050)).right;
        let off = m.compute_at(&fs, base + Duration::from_millis(1180)).right;
        assert!(on > 0);
        assert_eq!(off, 0);
    }

    #[test]
    fn helicopter_rotor_beat_pulses() {
        let mut fs = FlightState::new();
        fs.apply(IDX_ONGROUND_ANY, 0.0);
        fs.apply(IDX_ALPHA, 3.0);
        fs.apply(IDX_IS_HELI, 1.0);
        fs.apply(IDX_PROP_RPM, 360.0); // 6 Hz once-per-rev
        let mut m = default_mixer();
        let t0 = Instant::now();
        let mut levels = vec![];
        for i in 0..60 {
            fs.last_update = Some(t0 + Duration::from_millis(i * 16));
            levels.push(m.compute_at(&fs, t0 + Duration::from_millis(i * 16)).left);
        }
        let max = *levels.iter().max().unwrap();
        let min = *levels.iter().min().unwrap();
        assert!(max > 0 && min == 0, "rotor beat should pulse, got {min}..{max}");
    }

    #[test]
    fn brake_trigger_firmer_with_speed_and_locked_with_parking_brake() {
        let strength = |o: MixOutput| match o.left_trigger {
            Some(TriggerEffect::Feedback { strength, .. }) => strength,
            other => panic!("expected feedback, got {other:?}"),
        };
        let slow = strength(default_mixer().compute(&on_ground_rolling(2.0)));
        let fast = strength(default_mixer().compute(&on_ground_rolling(40.0)));
        assert!(fast > slow);
        let mut parked = on_ground_rolling(0.0);
        parked.apply(IDX_PARKING_BRAKE, 1.0);
        assert_eq!(strength(default_mixer().compute(&parked)), 8);

        let mut air = FlightState::new();
        air.apply(IDX_ONGROUND_ANY, 0.0);
        assert_eq!(default_mixer().compute(&air).left_trigger, Some(TriggerEffect::Off));
    }

    #[test]
    fn throttle_detent_and_reverse() {
        let mut fs = on_ground_rolling(0.0);
        match default_mixer().compute(&fs).right_trigger {
            Some(TriggerEffect::MultiFeedback { strengths }) => {
                assert_eq!(strengths[7], 1);
                assert_eq!(strengths[8], 6, "detent at zone 8 by default");
            }
            other => panic!("expected detent profile, got {other:?}"),
        }
        fs.apply(IDX_REVERSER, 1.0);
        assert!(matches!(default_mixer().compute(&fs).right_trigger, Some(TriggerEffect::Vibration { .. })));
    }

    #[test]
    fn triggers_disabled_means_untouched() {
        let cfg = TriggerConfig { enabled: false, ..TriggerConfig::default() };
        let mut m = EffectMixer::new(Gains::default(), cfg, None);
        let o = m.compute(&on_ground_rolling(10.0));
        assert_eq!((o.right_trigger, o.left_trigger), (None, None));
    }

    #[test]
    fn stale_data_goes_silent_and_releases_triggers() {
        let fs = on_ground_rolling(40.0);
        let mut m = default_mixer();
        let o = m.compute_at(&fs, Instant::now() + Duration::from_secs(3));
        assert_eq!((o.left, o.right), (0, 0));
        assert_eq!(o.right_trigger, Some(TriggerEffect::Off));
    }
}
