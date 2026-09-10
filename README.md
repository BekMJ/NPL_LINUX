# XHale — Embedded Firmware

Firmware for **XHale**, a portable breath-based carbon monoxide (CO) detector used for smoking
cessation monitoring. Runs on a Nordic **nRF52805** SoC and streams calibrated CO, temperature,
humidity, and pressure readings to a companion mobile app over Bluetooth Low Energy.

Companion apps: [XHALE-HEALTH-ANDROID](https://github.com/BekMJ/XHALE-HEALTH-ANDROID)

## What it does

- Samples an electrochemical CO sensor through the nRF52's **SAADC**, driven by a
  hardware **TIMER + PPI** chain so sampling continues without CPU involvement.
- Reads a **BME280** over **TWI (I²C)** for temperature, humidity, and barometric pressure —
  including the full fixed-point compensation routines derived from the sensor's factory
  calibration registers (`compensate_T_int32`, `compensate_H_int32`, `compensate_P_int32`).
- Exposes readings as a custom **GATT Environmental Sensing service** (see table below).
- Runs a **two-stage CO calibration**: continuous auto-zero baseline tracking, plus a span
  calibration against a 50 ppm reference gas. Calibration constants persist across power
  cycles in flash via **NVMC**.
- Reports out-of-range readings distinctly rather than clipping silently, so the app can
  show "High" instead of a misleading number.
- Designed for battery operation: `nrf_pwr_mgmt` idle handling, sleep on advertising timeout,
  and a hardware wake switch on **P0.20**.

## BLE service

Environmental Sensing Service, UUID `0x181A`:

| Characteristic | UUID | Notes |
|---|---|---|
| CO concentration | `0x2BD0` | ppm, calibrated |
| Temperature | `0x2A6E` | BME280, compensated |
| Humidity | `0x2A6F` | BME280, compensated |
| Pressure | `0x2A6D` | BME280, compensated |
| Status byte | `0x2BBB` | device state flags |
| Command byte | `0x2A9F` | app → device control |
| Test byte | `0x2AF4` | diagnostics |

## Hardware and toolchain

| | |
|---|---|
| SoC | Nordic nRF52805 |
| Board target | `pca10040e_nrf52805` |
| SoftDevice | S112 v7.0 (BLE peripheral) |
| SDK | Nordic nRF5 SDK |
| Toolchain | `arm-none-eabi-gcc` (GNU Arm Embedded) |
| Sensors | Electrochemical CO cell (analog, via SAADC) · Bosch BME280 (I²C) |

Built on Nordic's `ble_app_uart` SDK example as a project skeleton; the sensor drivers,
GATT service, calibration logic, and power management are project-specific.

## Building

```bash
cd pca10040e_nrf52805/s112/armgcc
make
```

Requires `GNU_INSTALL_ROOT` and the nRF5 SDK path set in the `Makefile`, and the S112
SoftDevice flashed before the application:

```bash
nrfjprog --program <path-to-s112_nrf52_7.0.1_softdevice.hex> --sectorerase
make flash
```

## Repository layout

```
main.c                                 application: sensors, calibration, BLE plumbing, power
ble_eco.c / ble_eco.h                  custom Environmental Sensing GATT service
pca10040e_nrf52805/s112/armgcc/        Makefile and linker script
pca10040e_nrf52805/s112/config/        sdk_config.h (SDK module enables)
```

## Calibration procedure

The constants at the top of `main.c` are set during bring-up:

1. Set `CO_PPM_UNCAL_AT_MAX` to `0x32`, flash, and run the N₂ flush to establish zero.
2. Apply the 50 ppm CO reference for ~50 s and read the raw value reported over BLE.
3. Replace `0x32` with that value and re-flash.

Auto-zero then tracks baseline drift continuously at runtime.

## Related publication

B. Weng, **B. Mijiddorj**, T. Beringer, L. Mijiddorj, Y. Yang, A. Ho, E. Hassan.
"An Environment-Adaptive Low-Power IoT Architecture for Portable Smoking Detection with
Real-Time Data Quality Assurance." 2026.
