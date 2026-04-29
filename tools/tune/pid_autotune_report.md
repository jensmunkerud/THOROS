# PID Autotune Report

Input log: logs/OSCILLATION.csv
Safe scale factor: 0.65

## Suggested Gains

| Axis | Kp | Ki | Kd | Safe Kp | Safe Ki | Safe Kd | Fit R^2 | Cost |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| pitch | 0.4660 | 21.8549 | 0.0000 | 0.3029 | 14.2057 | 0.0000 | 1.000 | 11.62364 |
| roll | 199.9757 | 0.0133 | 199.9931 | 129.9842 | 0.0086 | 129.9955 | 0.999 | 0.58866 |
| yaw | 119.1978 | 146.8297 | 135.7489 | 77.4786 | 95.4393 | 88.2368 | 1.000 | 23.95332 |

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

`0.3029/14.2057/0.0000/129.9842/0.0086/129.9955/77.4786/95.4393/88.2368`