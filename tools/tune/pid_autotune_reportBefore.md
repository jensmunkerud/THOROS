# PID Autotune Report

Input log: logs/OSCILLATION.csv
Safe scale factor: 0.65

## Suggested Gains

| Axis | Kp | Ki | Kd | Safe Kp | Safe Ki | Safe Kd | Fit R^2 | Cost |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| pitch | 0.4659 | 21.8548 | 0.0000 | 0.3029 | 14.2056 | 0.0000 | 1.000 | 11.62364 |
| roll | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.999 | 0.58703 |
| yaw | 2.0000 | 4.0000 | 1.0000 | 1.3000 | 2.6000 | 0.6500 | 1.000 | 23.95332 |

## Identified Plant

| Axis | Gain | Tau (s) | Delay (samples) |
|---|---:|---:|---:|
| pitch | 28.31166 | 19.99900 | 25 |
| roll | -0.01725 | 7.63964 | 25 |
| yaw | -0.38167 | 0.59395 | 0 |

## Notes

- No warnings.

## Firmware Payload

Safe first-flight payload string:

`0.3029/14.2056/0.0000/0.0000/0.0000/0.0000/1.3000/2.6000/0.6500`