# V1.0

## Hardware

- Diode should be added to USB power line
    - Without diode USB-LC6 will be damaged
    - Also without diode voltage from step-down regulator can get to USB power line (power line overvoltage)
    - Can be fixed by adding low dropout shottky diode to USB_VCC line


- Motor connector are bit wider then footprint
    - Connectors for motors a endstops are overlapping
    - Can be fixed by sanding off 0.5mm of material from back side of endstops connector (after that fits perfectly)

- Motor input voltage drop compensation is connection to 3.3V instead of VMOT
    - Compensation cannot be utilized

- Transil in stepdown regulator should be placed after fuse not before
    - When transil opens is triggered overcurrent protection of stepdow
    - Fuse is not activated by transil
