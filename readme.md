# wled-imu_QMI8658

WLED usermod driver for the QMI8658 6-axis IMU (accelerometer + gyroscope).

Provides IMU data to the [wled-motion_reactive](https://github.com/wled/wled-motion_reactive) usermod, which implements motion-reactive LED effects for staffs, wands, and other moving props.

## Hardware

The QMI8658 is a combined accelerometer/gyroscope connected over I²C.  Configure the I²C pins in WLED's LED Settings → Hardware page.

**I²C address**: `0x6B` (default) or `0x6A` — selectable by the SA0 pin.

## Configuration

All settings are in WLED's Usermod Settings page under **IMU_QMI8658**:

| Setting | Default | Description |
|---|---|---|
| enabled | false | Enable the driver |
| address | 0x6B | I²C address |
| update_interval_ms | 20 | Sensor poll interval (50 Hz) |
| filter_tau | 1.5 | Complementary filter time constant (seconds) |
| pitch_deg | 0 | Misalignment correction: rotation around X |
| roll_deg | 0 | Misalignment correction: rotation around Y |

### Misalignment correction

If the sensor is not perfectly aligned with the LED strip, gravity leaks into the wrong axes.
Set `pitch_deg` / `roll_deg` to compensate:

1. Hold the prop perfectly vertical (strip end pointing down).
2. Check the web UI info panel for Accel X/Y/Z readings.
3. The axis parallel to the strip should read ≈ ±9.8 m/s²; the others ≈ 0.
4. Adjust pitch/roll until the cross-axis leakage is minimised.

## Building

Add both this driver and `wled-motion_reactive` to your `custom_usermods` in `platformio_override.ini`:

```ini
[env:myboard]
extends = env:esp32dev
custom_usermods =
  ${env:esp32dev.custom_usermods}
  file:///path/to/wled-motion_reactive
  file:///path/to/wled-imu_QMI8658
```

Or reference published repos directly by URL once available.
