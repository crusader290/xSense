//! DualSense HID output-report construction and device handling.
//!
//! Two wire formats are supported:
//!   - USB:       report id 0x02, 63-byte "common" payload starting at byte 0.
//!   - Bluetooth: report id 0x31, 3-byte BT header, then the same common
//!                payload shifted by +2 bytes, then a little-endian CRC32
//!                over everything except the CRC itself, seeded with 0xA2.
//!
//! Reference: Linux kernel hid-playstation.c (Roderick Colenbrander),
//! nowrep/dualsensectl, Nielk1's DS4Windows/DSX trigger-effect gist.

use anyhow::{anyhow, Context, Result};
use hidapi::{BusType, HidApi, HidDevice};
use log::{debug, info, warn};

pub const VID_SONY: u16 = 0x054c;
pub const PID_DUALSENSE: u16 = 0x0ce6;
pub const PID_DUALSENSE_EDGE: u16 = 0x0df2;

/// USB output reports are 64 bytes on the wire (report id + 63-byte common
/// block, host-side buffer padded to 64). BT output reports are 78 bytes.
const USB_REPORT_LEN: usize = 64;
const BT_REPORT_LEN: usize = 78;

#[cfg(windows)]
const PERMISSION_HINT: &str = "Another app may be holding it exclusively (DS4Windows/DSX with HidHide, or Steam Input) - close it or whitelist this exe.";
#[cfg(not(windows))]
const PERMISSION_HINT: &str = "Do the udev rules grant access to hidraw? (see README)";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Connection {
    Usb,
    Bluetooth,
}

pub struct DualSense {
    device: HidDevice,
    pub connection: Connection,
    bt_seq: u8,
}

/// Adaptive trigger effects, encoded as the 11-byte descriptor the DualSense
/// expects (mode byte + 10 parameter bytes). Encodings follow Nielk1's
/// TriggerEffectGenerator (the de facto reference used by DSX/DS4Windows):
/// trigger travel is split into 10 zones (0 = released, 9 = fully pulled),
/// with an active-zone bitmask in bytes 1-2 and per-zone 3-bit strengths
/// packed little-endian into bytes 3-6.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerEffect {
    /// No resistance, no vibration.
    Off,
    /// Uniform resistance from zone `start` (0-9) to full pull, `strength` 1-8.
    Feedback { start: u8, strength: u8 },
    /// Independent resistance per zone, each 0 (none) to 8 (max). This is
    /// how detents are built: a step up in resistance at a given travel.
    MultiFeedback { strengths: [u8; 10] },
    /// Not used by the default profiles; available for custom effects.
    #[allow(dead_code)]
    /// A "wall" you push through between zones `start` (2-7) and `end`
    /// (start+1..=8), like a gun trigger break. `strength` 1-8.
    Weapon { start: u8, end: u8, strength: u8 },
    /// Trigger vibration from zone `start` (0-9) onward. `amplitude` 1-8,
    /// `frequency` in Hz (1-255).
    Vibration { start: u8, amplitude: u8, frequency: u8 },
}

impl TriggerEffect {
    /// Serialize into the 11-byte trigger effect descriptor used at byte 10
    /// (right) / byte 21 (left) of the common report block.
    pub fn to_bytes(self) -> [u8; 11] {
        fn pack(strengths: &[u8; 10]) -> (u16, u32) {
            let mut active: u16 = 0;
            let mut zones: u32 = 0;
            for (i, &s) in strengths.iter().enumerate() {
                if s > 0 {
                    let v = (s.min(8) - 1) as u32 & 0x07;
                    zones |= v << (3 * i);
                    active |= 1 << i;
                }
            }
            (active, zones)
        }
        fn from(start: u8, strength: u8) -> [u8; 10] {
            let mut s = [0u8; 10];
            let strength = strength.clamp(1, 8);
            for z in s.iter_mut().skip(start.min(9) as usize) {
                *z = strength;
            }
            s
        }

        let mut b = [0u8; 11];
        match self {
            TriggerEffect::Off => {
                b[0] = 0x05;
            }
            TriggerEffect::Feedback { start, strength } => {
                let (active, zones) = pack(&from(start, strength));
                b[0] = 0x21;
                b[1..3].copy_from_slice(&active.to_le_bytes());
                b[3..7].copy_from_slice(&zones.to_le_bytes());
            }
            TriggerEffect::MultiFeedback { strengths } => {
                let (active, zones) = pack(&strengths);
                if active == 0 {
                    return TriggerEffect::Off.to_bytes();
                }
                b[0] = 0x21;
                b[1..3].copy_from_slice(&active.to_le_bytes());
                b[3..7].copy_from_slice(&zones.to_le_bytes());
            }
            TriggerEffect::Weapon { start, end, strength } => {
                let start = start.clamp(2, 7);
                let end = end.clamp(start + 1, 8);
                let zones: u16 = (1 << start) | (1 << end);
                b[0] = 0x25;
                b[1..3].copy_from_slice(&zones.to_le_bytes());
                b[3] = strength.clamp(1, 8) - 1;
            }
            TriggerEffect::Vibration { start, amplitude, frequency } => {
                if frequency == 0 {
                    return TriggerEffect::Off.to_bytes();
                }
                let (active, zones) = pack(&from(start, amplitude));
                b[0] = 0x26;
                b[1..3].copy_from_slice(&active.to_le_bytes());
                b[3..7].copy_from_slice(&zones.to_le_bytes());
                b[9] = frequency;
            }
        }
        b
    }
}

/// Everything the mixer wants to push to the controller on a given tick.
#[derive(Debug, Clone, Copy, Default)]
pub struct OutputState {
    pub motor_left: u8,
    pub motor_right: u8,
    pub lightbar: (u8, u8, u8),
    pub player_leds: u8,
}

impl DualSense {
    /// Enumerate and open the first DualSense / DualSense Edge found.
    /// Bluetooth is tried first per project priority; if a BT-mode pad
    /// isn't found we fall back to whatever hidapi hands us (typically USB).
    pub fn open(api: &HidApi, prefer: Option<Connection>) -> Result<Self> {
        let mut candidates = vec![];
        for pid in [PID_DUALSENSE, PID_DUALSENSE_EDGE] {
            for dev in api.device_list().filter(|d| d.vendor_id() == VID_SONY && d.product_id() == pid) {
                candidates.push(dev.clone());
            }
        }
        if candidates.is_empty() {
            return Err(anyhow!(
                "no DualSense found (vid=0x{:04x}, pid=0x{:04x}/0x{:04x}). Is it paired/plugged in? {}",
                VID_SONY,
                PID_DUALSENSE,
                PID_DUALSENSE_EDGE,
                PERMISSION_HINT
            ));
        }

        // Prefer hidapi's reported bus type (reliable on Windows and modern
        // Linux). If it's unknown, fall back to probing the input report
        // size, which differs between transports (64 vs 78 bytes).
        let mut opened: Vec<(HidDevice, Connection)> = vec![];
        for info in &candidates {
            match info.open_device(api) {
                Ok(dev) => {
                    let conn = match info.bus_type() {
                        BusType::Bluetooth => Connection::Bluetooth,
                        BusType::Usb => Connection::Usb,
                        _ => detect_connection(&dev).unwrap_or(Connection::Usb),
                    };
                    debug!(
                        "found DualSense at path {:?} -> detected {:?}",
                        info.path(),
                        conn
                    );
                    opened.push((dev, conn));
                }
                Err(e) => warn!("could not open candidate device: {e}"),
            }
        }

        if opened.is_empty() {
            return Err(anyhow!("found DualSense in enumeration but failed to open any of them. {PERMISSION_HINT}"));
        }

        // Pick preferred transport if present, else first available.
        let idx = prefer
            .and_then(|want| opened.iter().position(|(_, c)| *c == want))
            .unwrap_or(0);
        let (device, connection) = opened.into_iter().nth(idx).unwrap();

        info!("using DualSense over {:?}", connection);
        Ok(Self { device, connection, bt_seq: 0 })
    }

    /// Build and write one output report reflecting `state`. Trigger effects
    /// are optional: `None` leaves that trigger's current effect untouched
    /// (its valid-flag bit stays clear), so disabling trigger features in
    /// the config never fights another app's trigger settings.
    pub fn write(&mut self, state: OutputState, right_trigger: Option<TriggerEffect>, left_trigger: Option<TriggerEffect>) -> Result<()> {
        let report = match self.connection {
            Connection::Usb => self.build_usb_report(state, right_trigger, left_trigger),
            Connection::Bluetooth => self.build_bt_report(state, right_trigger, left_trigger),
        };
        self.device
            .write(&report)
            .with_context(|| format!("hid write failed ({:?} report, {} bytes)", self.connection, report.len()))?;
        Ok(())
    }

    /// Fill the 47-byte "common" control block shared by both transports.
    /// `out[0]` here corresponds to valid_flag0 (i.e. caller writes this
    /// slice starting at the byte right after the report id).
    fn fill_common(common: &mut [u8], state: OutputState, right_trigger: Option<TriggerEffect>, left_trigger: Option<TriggerEffect>) {
        debug_assert!(common.len() >= 47);
        // valid_flag0: bit0 = COMPATIBLE_VIBRATION, bit1 = HAPTICS_SELECT,
        // bit2 = RIGHT_TRIGGER_MOTOR_ENABLE, bit3 = LEFT_TRIGGER_MOTOR_ENABLE.
        // Without bits 2/3 the controller ignores the trigger descriptors.
        common[0] = 0x03
            | if right_trigger.is_some() { 0x04 } else { 0 }
            | if left_trigger.is_some() { 0x08 } else { 0 };
        // valid_flag1: enable LEDs / lightbar control paths (mirrors pydualsense's
        // broad "0x57" rather than hand-picking every bit).
        common[1] = 0x57;
        common[2] = state.motor_right;
        common[3] = state.motor_left;
        // bytes 4-8 reserved/audio, leave zero
        // byte 9: mute LED - leave 0 (off/no override)
        // byte 10: right trigger effect starts... offsets below are relative
        // to this common block, which begins at byte 1 of the report-id-relative
        // "common" struct in the kernel (we already excluded valid_flag bytes
        // above at 0/1, so index math matches dualsensectl's layout: right
        // trigger descriptor at common[10..21], left at common[21..32]).
        if let Some(rt) = right_trigger {
            common[10..21].copy_from_slice(&rt.to_bytes());
        }
        if let Some(lt) = left_trigger {
            common[21..32].copy_from_slice(&lt.to_bytes());
        }
        // valid_flag2 at common[38]: nothing extra needed for classic rumble v1.
        common[38] = 0x00;
        // player LEDs at common[43], lightbar RGB at common[44..47]
        common[43] = state.player_leds;
        common[44] = state.lightbar.0;
        common[45] = state.lightbar.1;
        common[46] = state.lightbar.2;
    }

    fn build_usb_report(&self, state: OutputState, rt: Option<TriggerEffect>, lt: Option<TriggerEffect>) -> Vec<u8> {
        build_usb_report_raw(state, rt, lt)
    }

    fn build_bt_report(&mut self, state: OutputState, rt: Option<TriggerEffect>, lt: Option<TriggerEffect>) -> Vec<u8> {
        self.bt_seq = self.bt_seq.wrapping_add(1) & 0x0f;
        build_bt_report_raw(state, rt, lt, self.bt_seq)
    }
}

/// Free function version of USB report construction, independent of an open
/// device, so it's directly unit-testable.
pub fn build_usb_report_raw(state: OutputState, rt: Option<TriggerEffect>, lt: Option<TriggerEffect>) -> Vec<u8> {
    let mut report = vec![0u8; USB_REPORT_LEN];
    report[0] = 0x02;
    DualSense::fill_common(&mut report[1..1 + 47], state, rt, lt);
    report
}

/// Free function version of BT report construction (see above). `seq` is the
/// 4-bit rolling sequence counter the caller manages.
pub fn build_bt_report_raw(state: OutputState, rt: Option<TriggerEffect>, lt: Option<TriggerEffect>, seq: u8) -> Vec<u8> {
    let mut report = vec![0u8; BT_REPORT_LEN];
    report[0] = 0x31;
    report[1] = seq << 4;
    report[2] = 0x10;
    DualSense::fill_common(&mut report[3..3 + 47], state, rt, lt);

    let len = report.len();
    let crc = bt_crc32(&report[0..len - 4]);
    report[len - 4..].copy_from_slice(&crc.to_le_bytes());
    report
}

/// Sony's BT CRC32: seed the running CRC state with constant 0xA2 (as if it
/// were the first byte fed in), then IEEE CRC32 (same poly/reflection as
/// zlib / the kernel's crc32_le) over the payload, complemented on output --
/// which is exactly what crc32fast's Hasher does by default.
fn bt_crc32(payload: &[u8]) -> u32 {
    let mut hasher = crc32fast::Hasher::new();
    hasher.update(&[0xA2]);
    hasher.update(payload);
    hasher.finalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn usb_report_has_correct_length_and_ids() {
        let out = OutputState { motor_left: 200, motor_right: 100, lightbar: (10, 20, 30), player_leds: 0b100 };
        let r = build_usb_report_raw(out, None, None);
        assert_eq!(r.len(), USB_REPORT_LEN);
        assert_eq!(r[0], 0x02, "USB report id must be 0x02");
        // valid_flag0 at r[1], motor_right at r[3], motor_left at r[4]
        assert_eq!(r[1] & 0x03, 0x03, "COMPATIBLE_VIBRATION | HAPTICS_SELECT bits must be set");
        assert_eq!(r[3], 100, "motor_right offset wrong");
        assert_eq!(r[4], 200, "motor_left offset wrong");
        // lightbar at r[45..48] (1 + 44..47)
        assert_eq!(&r[45..48], &[10, 20, 30]);
    }

    #[test]
    fn bt_report_has_correct_length_header_and_crc() {
        let out = OutputState { motor_left: 50, motor_right: 75, lightbar: (0, 0, 0), player_leds: 0 };
        let r = build_bt_report_raw(out, None, None, 3);
        assert_eq!(r.len(), BT_REPORT_LEN);
        assert_eq!(r[0], 0x31, "BT report id must be 0x31");
        assert_eq!(r[1], 3 << 4, "seq_tag byte wrong");
        assert_eq!(r[2], 0x10, "tag byte wrong");
        // motor_right at 3+3=6? common block starts at index 3, valid_flag0/1
        // occupy common[0..2], so motor_right = common[2] = r[5], motor_left = r[6]
        assert_eq!(r[5], 75, "motor_right offset wrong for BT layout");
        assert_eq!(r[6], 50, "motor_left offset wrong for BT layout");

        // Verify the CRC really is what a receiver would recompute.
        let len = r.len();
        let expected_crc = bt_crc32(&r[0..len - 4]);
        let actual_crc = u32::from_le_bytes(r[len - 4..].try_into().unwrap());
        assert_eq!(actual_crc, expected_crc, "trailing CRC32 must match payload");
    }

    #[test]
    fn feedback_matches_nielk1_encoding() {
        // Feedback(position=3, strength=8): zones 3..=9 active, each 3-bit
        // value 7, packed little-endian.
        let b = TriggerEffect::Feedback { start: 3, strength: 8 }.to_bytes();
        assert_eq!(b[0], 0x21);
        let active = u16::from_le_bytes([b[1], b[2]]);
        assert_eq!(active, 0b11_1111_1000);
        let zones = u32::from_le_bytes([b[3], b[4], b[5], b[6]]);
        for z in 0..10 {
            let v = (zones >> (3 * z)) & 7;
            assert_eq!(v, if z >= 3 { 7 } else { 0 }, "zone {z}");
        }
        assert!(b[7..].iter().all(|&x| x == 0));
    }

    #[test]
    fn multi_feedback_encodes_per_zone() {
        let b = TriggerEffect::MultiFeedback { strengths: [1, 1, 1, 1, 1, 1, 1, 1, 6, 6] }.to_bytes();
        assert_eq!(b[0], 0x21);
        assert_eq!(u16::from_le_bytes([b[1], b[2]]), 0x3FF);
        let zones = u32::from_le_bytes([b[3], b[4], b[5], b[6]]);
        assert_eq!((zones >> 21) & 7, 0, "zone 7 strength 1 -> 0");
        assert_eq!((zones >> 24) & 7, 5, "zone 8 strength 6 -> 5");
        assert_eq!((zones >> 27) & 7, 5, "zone 9 strength 6 -> 5");
    }

    #[test]
    fn weapon_and_vibration_layout() {
        let w = TriggerEffect::Weapon { start: 4, end: 6, strength: 8 }.to_bytes();
        assert_eq!(w[0], 0x25);
        assert_eq!(u16::from_le_bytes([w[1], w[2]]), (1 << 4) | (1 << 6));
        assert_eq!(w[3], 7);

        let v = TriggerEffect::Vibration { start: 0, amplitude: 8, frequency: 20 }.to_bytes();
        assert_eq!(v[0], 0x26);
        assert_eq!(u16::from_le_bytes([v[1], v[2]]), 0x3FF);
        assert_eq!(v[9], 20, "frequency lives at byte 9");
    }

    #[test]
    fn trigger_off_is_all_zero_except_type_byte() {
        let bytes = TriggerEffect::Off.to_bytes();
        assert_eq!(bytes[0], 0x05);
        assert!(bytes[1..].iter().all(|&b| b == 0));
    }

    #[test]
    fn trigger_valid_flags_follow_option() {
        let r = build_usb_report_raw(OutputState::default(), None, None);
        assert_eq!(r[1], 0x03, "no trigger bits when triggers unmanaged");
        let fx = TriggerEffect::Feedback { start: 0, strength: 4 };
        let r = build_usb_report_raw(OutputState::default(), Some(fx), None);
        assert_eq!(r[1], 0x07);
        assert_eq!(&r[11..22], &fx.to_bytes(), "right trigger at common[10..21]");
        let r = build_usb_report_raw(OutputState::default(), None, Some(fx));
        assert_eq!(r[1], 0x0B);
        assert_eq!(&r[22..33], &fx.to_bytes(), "left trigger at common[21..32]");
    }
}

/// Heuristic transport detection: read one input report and look at its
/// length. USB DualSense input reports are 64 bytes; Bluetooth ones are 78.
fn detect_connection(device: &HidDevice) -> Result<Connection> {
    let mut buf = [0u8; 96];
    let _ = device.set_blocking_mode(false);
    // A controller that's idle may not have a report queued yet; try a few
    // times with tiny sleeps rather than trusting a single read.
    for _ in 0..5 {
        match device.read_timeout(&mut buf, 50) {
            Ok(n) if n >= BT_REPORT_LEN - 2 => return Ok(Connection::Bluetooth),
            Ok(n) if n > 0 => return Ok(Connection::Usb),
            _ => std::thread::sleep(std::time::Duration::from_millis(20)),
        }
    }
    // Default assumption if we couldn't read anything (still lets writes go
    // out; USB is the safer default since its report is a strict subset).
    Ok(Connection::Usb)
}
