# dualsense_xplane

Real haptic feedback for X-Plane 12 on Linux, using a PS5 DualSense controller. No HOTAS or force-feedback rig required, this drives the controller you already have.

X-Plane has no native DualSense support, so this reads flight data over UDP and writes HID output reports straight to the controller over `hidraw`. No Windows SDK, no Steam Input, no audio-routing tricks. Works over both Bluetooth and USB.

Write-up of how it works and the bugs along the way: _[here](https://abdullahwaqar.com/blog/building-dualsense-haptics-xplane)_

```mermaid
flowchart LR
    A[X-Plane 12<br/>UDP :49000] -->|RREF stream| B[Parse]
    B --> C[Effect Mixer]
    C -->|HID report| D[DualSense<br/>USB or B1luetooth]
```

## What it feels like

Silent at cruise and at idle. Speaks up for:

- **Runway/taxi rumble** — scales with groundspeed once you're actually rolling (nothing below ~6kt)
- **Touchdown thump** — one-shot, severity driven by vertical speed and g-load at the moment of contact
- **Stall buffet** — ramps in only within a couple degrees of stall AoA, or when the stall-warning annunciator fires
- **Overspeed buzz** — discrete buzz when `over_vne` trips
- **Gear/flap bumps** — short transient on position change
- **Turbulence** — proxy off normal load factor deviation, with a deadband so ordinary sim jitter doesn't count
- **Lightbar** — doubles as an AoA-margin indicator, green with margin, red as you approach stall

Everything continuous has a deadzone. If nothing's actually happening, the controller is quiet. That took a couple of real bugs to get right (see the blog post).

## Status

Working, tested against real hardware over Bluetooth. Byte-level report construction, the Bluetooth CRC, and the UDP dataref parsing are all unit tested (`cargo test`, 11 tests). Not yet done, on purpose:

- Adaptive trigger effects are encoded and tested (`hid::TriggerEffect`) but not wired into the main loop. Throttle detent / G-loaded elevator feel is the natural next step.
- Turbulence is a `g_nrml`-deviation proxy, not real wind/gust datarefs.
- No per-aircraft stall AoA yet, `stall_alpha_deg` defaults to 14°. Wire up `sim/aircraft/overflow/acf_stall_warn_alpha` if you want it exact.

## Build

```bash
cargo build --release
```

## Controller permissions

`hidraw` is root-only by default. `uaccess`-based udev rules (the usual modern approach) did not reliably grant access in testing on a KDE/Wayland/sddm session even with an active seat, so this uses a static group instead:

```bash
sudo groupadd -f dualsense
sudo usermod -aG dualsense $USER

sudo cp udev/99-dualsense-xplane.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Log out and back in for the group membership to take effect, or use `sg dualsense -c "cargo run"` to test in your current session without logging out.

If you're on Bluetooth and the rule doesn't seem to take, check that your controller's `hidraw` node actually reports a `KERNELS` value like `0005:054C:0CE6.0016` (bus:VID:PID.instance), not `idVendor`/`idProduct` attributes, those only exist on the USB descriptor chain and won't be present for a BT-paired pad:

```bash
udevadm info -a -n /dev/hidraw8 | grep KERNELS
```

## X-Plane setup

Nothing to configure in-sim. The daemon sends its own `RREF` subscription requests to UDP port 49000 on startup, which X-Plane always listens on. Just make sure the daemon's `local_port` in `config.toml` doesn't collide with a port X-Plane has already claimed for its own networking:

```bash
sudo lsof -i UDP | grep -i main
```

## Run

```bash
sg dualsense -c "cargo run -- config.toml"
```

`RUST_LOG=debug` for per-dataref and per-report detail.

## Config

All gains live in `config.toml`, no recompiling needed:

```toml
connection = "bluetooth"   # "bluetooth" | "usb" | "auto"
tick_hz = 60

[gains]
ground_rumble = 0.35
touchdown = 1.0
stall_buffet = 0.8
turbulence = 0.5
overspeed = 0.9
gear_flap = 0.4
engine_rumble = 0.0   # off by default, constant idle hum isn't the goal here
```

## Architecture

- `xplane.rs` — RREF UDP request/response client. One thread loops on `recv()` and feeds shared flight state.
- `state.rs` — dataref index registry plus edge-detection for touchdown/gear/flap transients.
- `effects.rs` — pure function from flight state to `(left, right, lightbar)`. No I/O, deadzone and threshold logic lives here.
- `hid.rs` — DualSense USB (`0x02`) and Bluetooth (`0x31` + CRC32) wire format, plus adaptive-trigger descriptor encoding.
- `config.rs` — TOML config with per-effect gains.
- `main.rs` — one UDP-receive thread, one HID-write loop on the main thread at `tick_hz`.

## Reference material this was built from

- Linux kernel `hid-playstation.c` (classic rumble via FF, valid_flag bits)
- `nowrep/dualsensectl` (canonical Linux reference implementation, BT CRC32 handling)
- Nielk1's `TriggerEffectGenerator` (adaptive trigger effect byte-packing)
- X-Plane's documented `RREF` / `dref_struct_in` UDP protocol
