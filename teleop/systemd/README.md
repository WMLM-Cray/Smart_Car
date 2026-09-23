# Car service overrides

These are the service overrides used for the September 2026 car setup.
The paths under `systemd/` mirror `/etc/systemd/system/` on the car.

- `racecar-chassis.service.d/wait-serial.conf` waits for stable serial device
  nodes before starting the chassis driver.
- `racecar-policy.service.d/speed-loop.conf` starts the measured 0.7 m/s
  closed loop policy. It relies on the existing `racecar-policy.service` unit
  and on `runs/candidate_060.npz` being present on the car.

After installing or changing an override on the car, reload systemd and
restart the corresponding service. These files record the intended boot
configuration; the car was offline when this repository snapshot was pushed.
