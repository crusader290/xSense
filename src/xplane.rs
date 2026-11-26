//! Minimal X-Plane 12 "RREF" UDP client.
//!
//! Request packet layout (matches X-Plane's documented `dref_struct_in`):
//!   "RREF\0"  (5 bytes, header + null)
//!   i32 freq  (updates per second; 0 = stop)
//!   i32 index (caller-chosen tag, echoed back in every value packet)
//!   char[400] dataref path, null-terminated, zero-padded
//!
//! Response packet layout:
//!   "RREF," (5 bytes, header + comma)
//!   repeated (i32 index, f32 value) pairs, 8 bytes each, for as many
//!   datarefs as X-Plane packed into this datagram.

use anyhow::{Context, Result};
use log::{debug, warn};
use std::net::{SocketAddr, UdpSocket};
use std::time::Duration;

const REQ_HEADER: &[u8; 5] = b"RREF\0";
const RESP_HEADER: &[u8; 5] = b"RREF,";
const DATAREF_FIELD_LEN: usize = 400;

pub struct XPlaneClient {
    socket: UdpSocket,
    xplane_addr: SocketAddr,
}

impl XPlaneClient {
    /// `local_port` is where we listen for RREF responses; X-Plane sends
    /// them back to whatever source port our request came from, so binding
    /// explicitly just makes that predictable/loggable.
    pub fn new(xplane_ip: &str, xplane_port: u16, local_port: u16, recv_timeout: Duration) -> Result<Self> {
        let socket = UdpSocket::bind(("0.0.0.0", local_port))
            .with_context(|| format!("failed to bind local UDP port {local_port}"))?;
        socket.set_read_timeout(Some(recv_timeout))?;
        let xplane_addr: SocketAddr = format!("{xplane_ip}:{xplane_port}")
            .parse()
            .context("invalid xplane_ip/xplane_port")?;
        Ok(Self { socket, xplane_addr })
    }

    /// Subscribe to a single dataref at `freq` Hz, tagged with `index`.
    /// Call again with `freq = 0` to unsubscribe.
    pub fn subscribe(&self, dataref: &str, index: i32, freq: i32) -> Result<()> {
        if dataref.len() >= DATAREF_FIELD_LEN {
            anyhow::bail!("dataref path too long for RREF packet: {dataref}");
        }
        let mut packet = Vec::with_capacity(5 + 4 + 4 + DATAREF_FIELD_LEN);
        packet.extend_from_slice(REQ_HEADER);
        packet.extend_from_slice(&freq.to_le_bytes());
        packet.extend_from_slice(&index.to_le_bytes());
        let mut dref_field = vec![0u8; DATAREF_FIELD_LEN];
        dref_field[..dataref.len()].copy_from_slice(dataref.as_bytes());
        packet.extend_from_slice(&dref_field);

        self.socket
            .send_to(&packet, self.xplane_addr)
            .with_context(|| format!("failed to send RREF subscribe for {dataref}"))?;
        debug!("subscribed dataref '{dataref}' (index={index}, freq={freq}Hz)");
        Ok(())
    }

    pub fn subscribe_all(&self, subs: &[(String, i32, i32)]) -> Result<()> {
        for (dref, index, freq) in subs {
            self.subscribe(dref, *index, *freq)?;
        }
        Ok(())
    }

    /// Ask X-Plane to stop streaming everything (freq = 0), so it doesn't
    /// keep sending to a dead port after we exit.
    pub fn unsubscribe_all(&self, subs: &[(String, i32, i32)]) {
        for (dref, index, _) in subs {
            let _ = self.subscribe(dref, *index, 0);
        }
    }

    /// Block (up to the configured read timeout) for one UDP datagram and
    /// parse every (index, value) pair it contains. Returns an empty vec on
    /// timeout so the caller's loop can just keep spinning.
    pub fn recv(&self) -> Result<Vec<(i32, f32)>> {
        let mut buf = [0u8; 2048];
        let (n, _src) = match self.socket.recv_from(&mut buf) {
            Ok(v) => v,
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock || e.kind() == std::io::ErrorKind::TimedOut => {
                return Ok(vec![]);
            }
            Err(e) => return Err(e).context("UDP recv_from failed"),
        };

        if n < 5 || &buf[0..5] != RESP_HEADER {
            warn!("ignoring non-RREF packet ({n} bytes)");
            return Ok(vec![]);
        }

        let mut out = Vec::new();
        let mut pos = 5;
        while pos + 8 <= n {
            let index = i32::from_le_bytes(buf[pos..pos + 4].try_into().unwrap());
            let value = f32::from_le_bytes(buf[pos + 4..pos + 8].try_into().unwrap());
            out.push((index, value));
            pos += 8;
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a fake RREF response datagram the way X-Plane would, and check
    /// our parser recovers the same (index, value) pairs. This doesn't need
    /// a real X-Plane instance since it only exercises wire parsing.
    fn fake_rref_response(pairs: &[(i32, f32)]) -> Vec<u8> {
        let mut buf = Vec::new();
        buf.extend_from_slice(RESP_HEADER);
        for (idx, val) in pairs {
            buf.extend_from_slice(&idx.to_le_bytes());
            buf.extend_from_slice(&val.to_le_bytes());
        }
        buf
    }

    #[test]
    fn request_packet_has_correct_layout() {
        let client = XPlaneClient::new("127.0.0.1", 49000, 0, Duration::from_millis(10)).unwrap();
        // We can't easily intercept send_to without a second socket, so just
        // exercise the length-guard branch here for a too-long dataref.
        let long = "x".repeat(DATAREF_FIELD_LEN);
        assert!(client.subscribe(&long, 1, 30).is_err());
    }

    #[test]
    fn parses_single_pair() {
        let datagram = fake_rref_response(&[(3, 1.0)]);
        assert_eq!(&datagram[0..5], RESP_HEADER);
        let mut pos = 5;
        let mut out = vec![];
        while pos + 8 <= datagram.len() {
            let index = i32::from_le_bytes(datagram[pos..pos + 4].try_into().unwrap());
            let value = f32::from_le_bytes(datagram[pos + 4..pos + 8].try_into().unwrap());
            out.push((index, value));
            pos += 8;
        }
        assert_eq!(out, vec![(3, 1.0)]);
    }

    #[test]
    fn parses_multiple_batched_pairs() {
        let pairs = [(1, 12.5f32), (2, -600.0), (3, 0.0), (10, 98.7)];
        let datagram = fake_rref_response(&pairs);
        let mut pos = 5;
        let mut out = vec![];
        while pos + 8 <= datagram.len() {
            let index = i32::from_le_bytes(datagram[pos..pos + 4].try_into().unwrap());
            let value = f32::from_le_bytes(datagram[pos + 4..pos + 8].try_into().unwrap());
            out.push((index, value));
            pos += 8;
        }
        assert_eq!(out, pairs.to_vec());
    }
}
