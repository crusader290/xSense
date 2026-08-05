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
use hidapi::{HidApi, HidDevice};
use log::{debug, info, warn};

pub const VID_SONY: u16 = 0x054c;
pub const PID_DUALSENSE: u16 = 0x0ce6;
pub const PID_DUALSENSE_EDGE: u16 = 0x0df2;

/// USB output reports are 64 bytes on the wire (report id + 63-byte common
/// block, host-side buffer padded to 64). BT output reports are 78 bytes.
const USB_REPORT_LEN: usize = 64;
const BT_REPORT_LEN: usize = 78;

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

/// Effect type bytes for the adaptive trigger 11-byte descriptor.
/// (Nielk1 TriggerEffectGenerator; dualsensectl trigger subcommands.)
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)] // Feedback/Vibration are Stage-4 (adaptive trigger) effects, wired in later
pub enum TriggerEffect {
    Off,
    /// Constant resistance starting at `start_pos` (0-9) with `force` (0-255).
    Feedback { start_pos: u8, force: u8 },
    /// Simple vibration/rumble on the trigger itself.
    Vibration { start_pos: u8, amplitude: u8, frequency: u8 },
}

impl TriggerEffect {
    /// Serialize into the 11-byte trigger effect descriptor used at byte 11
    /// (right) / byte 22 (left) of the common report block.
    fn to_bytes(self) -> [u8; 11] {
        let mut b = [0u8; 11];
        match self {
            TriggerEffect::Off => {
                b[0] = 0x05; // "Off" / no-resistance mode
            }
            TriggerEffect::Feedback { start_pos, force } => {
                b[0] = 0x21; // "Feedback" (constant resistance region)
                // Bytes 1-2: 10-bit zone bitmap (which of the 10 trigger
                // zones are "active"). Bytes 3-9: 7 force-parameter slots,
                // one per active zone from start_pos onward. We use the
                // simplified single-region form dualsensectl exposes: start
                // position selects where resistance begins, force is applied
                // uniformly from there to full extension.
                let start = start_pos.min(9);
                let region_bitmap: u16 = 0xFFFFu16 << start;
                b[1] = (region_bitmap & 0xFF) as u8;
                b[2] = ((region_bitmap >> 8) & 0xFF) as u8;
                for (i, slot) in b[3..10].iter_mut().enumerate() {
                    if (i as u8) >= start {
                        *slot = force;
                    }
                }
            }
            TriggerEffect::Vibration { start_pos, amplitude, frequency } => {
                b[0] = 0x26; // "Vibration"
                let start = start_pos.min(9);
                let region_bitmap: u16 = 0xFFFFu16 << start;
                b[1] = (region_bitmap & 0xFF) as u8;
                b[2] = ((region_bitmap >> 8) & 0xFF) as u8;
                for (i, slot) in b[3..10].iter_mut().enumerate() {
                    if (i as u8) >= start {
                        *slot = amplitude;
                    }
                }
                b[10] = frequency;
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
                "no DualSense found (vid=0x{:04x}, pid=0x{:04x}/0x{:04x}). Is it paired/plugged in, and do udev rules grant access to hidraw?",
                VID_SONY, PID_DUALSENSE, PID_DUALSENSE_EDGE
            ));
        }

        // hidapi on Linux doesn't expose transport directly; we detect USB vs
        // BT by probing the device's actual input report size after opening,
        // which differs (64 vs 78 bytes) between the two transports.
        let mut opened: Vec<(HidDevice, Connection)> = vec![];
        for info in &candidates {
            match info.open_device(api) {
                Ok(dev) => {
                    let conn = detect_connection(&dev).unwrap_or(Connection::Usb);
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
            return Err(anyhow!("found DualSense in enumeration but failed to open any of them (permissions? check udev rules)"));
        }

        // Pick preferred transport if present, else first available.
        let idx = prefer
            .and_then(|want| opened.iter().position(|(_, c)| *c == want))
            .unwrap_or(0);
        let (device, connection) = opened.into_iter().nth(idx).unwrap();

        info!("using DualSense over {:?}", connection);
        Ok(Self { device, connection, bt_seq: 0 })
    }

    /// Build and write one output report reflecting `state`, with the given
    /// per-trigger effects (usually Off unless you're doing Stage 4 trigger work).
    pub fn write(&mut self, state: OutputState, right_trigger: TriggerEffect, left_trigger: TriggerEffect) -> Result<()> {
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
    fn fill_common(common: &mut [u8], state: OutputState, right_trigger: TriggerEffect, left_trigger: TriggerEffect) {
        debug_assert!(common.len() >= 47);
        // valid_flag0: bit0 = COMPATIBLE_VIBRATION, bit1 = HAPTICS_SELECT
        common[0] = 0x03;
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
        common[10..21].copy_from_slice(&right_trigger.to_bytes());
        common[21..32].copy_from_slice(&left_trigger.to_bytes());
        // valid_flag2 at common[38]: nothing extra needed for classic rumble v1.
        common[38] = 0x00;
        // player LEDs at common[43], lightbar RGB at common[44..47]
        common[43] = state.player_leds;
        common[44] = state.lightbar.0;
        common[45] = state.lightbar.1;
        common[46] = state.lightbar.2;
    }

    fn build_usb_report(&self, state: OutputState, rt: TriggerEffect, lt: TriggerEffect) -> Vec<u8> {
        build_usb_report_raw(state, rt, lt)
    }

    fn build_bt_report(&mut self, state: OutputState, rt: TriggerEffect, lt: TriggerEffect) -> Vec<u8> {
        self.bt_seq = self.bt_seq.wrapping_add(1) & 0x0f;
        build_bt_report_raw(state, rt, lt, self.bt_seq)
    }
}

/// Free function version of USB report construction, independent of an open
/// device, so it's directly unit-testable.
pub fn build_usb_report_raw(state: OutputState, rt: TriggerEffect, lt: TriggerEffect) -> Vec<u8> {
    let mut report = vec![0u8; USB_REPORT_LEN];
    report[0] = 0x02;
    DualSense::fill_common(&mut report[1..1 + 47], state, rt, lt);
    report
}

/// Free function version of BT report construction (see above). `seq` is the
/// 4-bit rolling sequence counter the caller manages.
pub fn build_bt_report_raw(state: OutputState, rt: TriggerEffect, lt: TriggerEffect, seq: u8) -> Vec<u8> {
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
        let r = build_usb_report_raw(out, TriggerEffect::Off, TriggerEffect::Off);
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
        let r = build_bt_report_raw(out, TriggerEffect::Off, TriggerEffect::Off, 3);
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
    fn trigger_feedback_encodes_force_from_start_position() {
        let bytes = TriggerEffect::Feedback { start_pos: 3, force: 200 }.to_bytes();
        assert_eq!(bytes.len(), 11);
        assert_eq!(bytes[0], 0x21);
        // force slots live at bytes[3..10] (7 zones); zones before start_pos
        // stay 0, zones from start_pos onward carry the force value.
        assert_eq!(bytes[3], 0, "zone 0 should be untouched");
        assert_eq!(bytes[3 + 3], 200, "zone 3 == start_pos should carry force");
        assert_eq!(bytes[9], 200, "zone 6, last force slot, should carry force");
    }

    #[test]
    fn trigger_off_is_all_zero_except_type_byte() {
        let bytes = TriggerEffect::Off.to_bytes();
        assert_eq!(bytes[0], 0x05);
        assert!(bytes[1..].iter().all(|&b| b == 0));
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
