# dualsense_xplane

Real haptic feedback for X-Plane 12 on Linux and Windows, using a PS5 DualSense controller. No HOTAS or force-feedback rig required, this drives the controller you already have.

X-Plane has no native DualSense support, so this reads flight data over UDP and writes HID output reports straight to the controller (`hidraw` on Linux, the native HID API on Windows). No Steam Input, no audio-routing tricks. Works over both Bluetooth and USB.

> **Fork note:** this fork adds Windows support, prebuilt Windows releases, adaptive triggers (throttle detent, brakes, stick shaker) and a set of new rumble effects. See [Releases](../../releases) for the Windows build.

Write-up of how it works and the bugs along the way: _[here](https://abdullahwaqar.com/blog/building-dualsense-haptics-xplane)_

```mermaid
flowchart LR
    A[X-Plane 12<br/>UDP :49000] -->|RREF stream| B[Parse]
    B --> C[Effect Mixer]
    C -->|HID report| D[DualSense<br/>USB or Bluetooth]
```

## What it feels like

Silent at cruise and at idle. The two rumble motors are routed separately: the heavy left motor carries thumps and ground roll, the light right motor carries buzzes and buffets, so overlapping effects stay distinguishable.

**Heavy motor (left)**
- **Runway/taxi rumble**: scales with groundspeed once you're actually rolling (nothing below ~6kt)
- **Surface texture**: smooth on concrete/asphalt; irregular bumps on grass, dirt and gravel; rolling hull slap on water
- **Touchdown thump**: one-shot, severity driven by vertical speed and g-load at the moment of contact
- **Nose/tail wheel thump**: lighter, sharper bump when the nose wheel comes down after the mains
- **Engine light-off / shutdown**: a shove when an engine starts, a softer shudder when it stops
- **Gear-down rumble**: faint airframe rumble with the gear out, growing with airspeed
- **Rotor beat** (helicopters): pulses at the main rotor's once-per-rev rate

**Light motor (right)**
- **Stall buffet**: ramps in only within a couple degrees of the aircraft's stall-warning AoA, or when the stall-warning annunciator fires
- **Speedbrake buffet**: spoiler deployment × airspeed², so slow deployment is quiet and high-speed deployment shakes
- **Overspeed**: steady buzz over Vne; pulsing buzz over flap (Vfe) or gear (Vle) limit speed
- **Autopilot disconnect**: double tap when the AP servos disengage
- **Gear/flap bumps**: short transient on position change

**Both**
- **Turbulence**: jolts in normal and lateral g, high-pass filtered so a steady turn or pull-up doesn't register, scaled by X-Plane 12's weather turbulence at your altitude

**Adaptive triggers**
- **R2 throttle detent**: light resistance, then a resistance step at a configurable position (default: the climb/TOGA end of travel)
- **R2 reverse**: buzzes while thrust reversers are deployed
- **L2 brakes**: firmer as groundspeed rises; locked solid with the parking brake set
- **Stick shaker**: both triggers shake while the stall warning is active

Trigger zones are positions along the trigger's travel, so the detents only line up with throttle settings if you bind R2 to the throttle axis (and L2 to brakes) in X-Plane's joystick settings. Set `[triggers] enabled = false` to leave the triggers alone.

**Lightbar** doubles as an AoA-margin indicator: green with margin, red as you approach stall, dim blue when no data is arriving from X-Plane.

Everything continuous has a deadzone. If nothing's actually happening, the controller is quiet. That took a couple of real bugs to get right (see the blog post).

## Status

Upstream rumble effects were tested against real hardware over Bluetooth on Linux. The effects and adaptive triggers added in this fork are unit tested (`cargo test`, 34 tests covering report bytes, trigger encoding, effect routing and edge detection) but **not yet tested on a real controller**, so the gains and trigger strengths may need tuning.

- Stall AoA comes from each aircraft's `acf_stall_warn_alpha`, falling back to 14°. Override with `stall_alpha_deg` in `config.toml`.
- If X-Plane stops sending data for 1.5 s (paused, quit), output goes silent and the triggers are released.
- The daemon unsubscribes from X-Plane on exit.

## Windows

1. Download `xsense-v<version>-windows-x64.zip` from [Releases](../../releases) and unzip it anywhere.
2. Pair the DualSense over Bluetooth (Settings → Bluetooth & devices → hold **PS + Create** until the light flashes) or plug it in with USB.
3. Start X-Plane 12, then run `dualsense-xplane.exe`. It reads `config.toml` from the same folder. If Windows Firewall asks, allow it on private networks: it only talks UDP to X-Plane.

Notes:
- No drivers or admin rights needed; it uses the standard Windows HID API.
- If it says the controller can't be opened, another app is holding it exclusively. Common culprits are DS4Windows/DSX with HidHide (whitelist `dualsense-xplane.exe` or close them) and Steam with PlayStation controller support enabled for X-Plane.
- `set RUST_LOG=debug` before running for per-dataref and per-report detail.

## Build

```bash
cargo build --release
```

Linux needs `libudev-dev` (Debian/Ubuntu) for the hidapi build. On Windows, a plain `cargo build --release` with the MSVC toolchain works; the C runtime is statically linked (`.cargo/config.toml`), so the `.exe` has no runtime dependencies. Every push to `main` runs `.github/workflows/release.yml`, which tests on Linux and Windows. If no release exists yet for the `version` in `Cargo.toml`, it tags that commit `v<version>` and publishes the Windows zip; bump the version to cut a new release.

## Controller permissions (Linux)

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
sudo lsof -i UDP | grep -i main          # Linux
netstat -ano -p udp | findstr 52010      # Windows
```

## Run (Linux)

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
speedbrake_buffet = 0.5
surface_rough = 0.35
gear_buffet = 0.25
engine_start = 0.6
rotor_beat = 0.25     # helicopters only
prop_beat = 0.0       # off by default
ap_disconnect = 0.6
config_overspeed = 0.6
nose_touchdown = 0.6

[triggers]
enabled = true
throttle = true
throttle_detent_zone = 8      # 1-9 along R2's travel
throttle_detent_strength = 6  # 1-8
reverse_buzz = true
brakes = true
stall_shaker = true
stall_shaker_hz = 20
```

Every gain is 0.0–1.0; set any to 0.0 to turn that effect off. See the comments in `config.toml` for what each one does.

## Architecture

- `xplane.rs` — RREF UDP request/response client. One thread loops on `recv()` and feeds shared flight state.
- `state.rs` — dataref index registry plus edge detection for touchdown, nose-wheel, gear/flap, engine start/stop and AP-disconnect transients; weather-layer turbulence interpolation.
- `effects.rs` — flight state → left/right motor levels, lightbar and trigger effects. No I/O; deadzone, routing and threshold logic lives here.
- `hid.rs` — DualSense USB (`0x02`) and Bluetooth (`0x31` + CRC32) wire format, adaptive-trigger descriptor encoding (Nielk1 format), transport detection via hidapi bus type.
- `config.rs` — TOML config with per-effect gains.
- `main.rs` — one UDP-receive thread, one HID-write loop on the main thread at `tick_hz`.

## Reference material this was built from

- Linux kernel `hid-playstation.c` (classic rumble via FF, valid_flag bits)
- `nowrep/dualsensectl` (canonical Linux reference implementation, BT CRC32 handling)
- Nielk1's `TriggerEffectGenerator` (adaptive trigger effect byte-packing)
- X-Plane's documented `RREF` / `dref_struct_in` UDP protocol
