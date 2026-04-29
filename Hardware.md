# Hardware
## Control
 - Teensy 4.1 MCU
 - 1000 Hz control loop
 - Custom per-tick datalogging system with 5-11us latency on 512 byte packets.
## Sensor
 - Extremely EMI resilient analog circuitry
 - Two IE2-1024 encoders (L/R)
 - LSM6DSV16X IMU located 23mm left, 11mm behind CoG with all values aligned to project conventions before reaching software.
 - Custom IR wall sensors with +/-3 degree half angle emitters and 20 degree phototransistor receivers
    - Individual (r,f) location and unit vector relative heading available in software.
 - One left, one right, and two forward facing wall sensors. The forward facing sensors sit on the left and right side of the vehicle.
 - Log-amp based front wall sensors.
## Power Systems
 - Maxon RE 8 347723 2.4V fan motor
 - DRV8871 H-bridge motor drivers for L/R wheel motors
 - 2x Faulhaber 1717T006SR drive motors
 - 17:56 gear ratio
 - 25mm wheels, 17.2mm wheel hubs with solid rubber 20 durometer tires
	- 4 wheels, 2 left/2 right, shared pinion per bank
	- Contact patches centered +/-40mm right, +/-14.75mm forward
## Inertial Data
 - Drivetrain inertia (Wheels, gears, and rotor) in wheel frame of 240e-9 kg*m²
 - Yaw inertia of 220e-6 kg*m²
 - Vehicle mass of 140g
# Design Philosophy
 - Use relatively standard podium-grade hardware with extremely robust software to give top-3 grade performance
 - Heavy reliance on robust datalogging to characterize and tune systems.