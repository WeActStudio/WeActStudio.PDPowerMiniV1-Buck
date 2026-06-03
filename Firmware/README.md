> After upgrading, please manually restore the factory settings to ensure the device can continue to operate abnormally.

## Setting Introduction
1. `Brightness` – Brightness: Adjustable from 0–100%; 0–10% corresponds to ultra-low brightness.
2. `Reverse Dis` – Reverse Display: 0 = Normal, 1 = 180° rotation. Setting takes effect after power cycle.
3. `OCP Enable` – Over-Current Protection: 0 = Disabled, 1 = Enabled. Output shuts down after approximately 200 ms when current exceeds the set value.
4. `OC Discharge` – Over-Current Discharge: 0 = Disabled, 1 = Enabled. <small>(V1.0.0.2 and above)</small>
5. `AUTO OUT` – Auto Output: 0 = Disabled, 1 = Enabled. Output automatically enables after power-on.
6. `Filter.Win` – Output Filter Window: Adjustable from 1–200, default value 50. <small>(V1.0.0.3 and above)</small>
7. `Output Comp` – Output Compensation: 0 = Disabled, 1 = Enabled. <small>(V1.0.0.5 and above)</small>
8. `UART Mode` – UART Mode: 0 = Disabled, 1 = Enabled. Setting takes effect after power cycle. <small>(V1.0.0.5 and above)</small>
9. `UART Baudrate` – UART Baudrate: 0-6, respectively corresponding to 9600, 19200, 38400, 57600, 115200, 230400, 460800. <small>(V1.0.0.5 and above)</small>
10. `Factory Reset` – Factory Reset: Set to 1 and exit the settings menu to restore factory defaults (calibration values will not be cleared).
11. `Offset Enable` – Calibration Enable: 0 = Disabled, 1 = Enabled. Factory default is 1 (calibrated).
> UART pins are assigned to DP and DM of Type‑C connector:
DP → TX, DM → RX, 3.3 V logic level.  
`Note`: The external UART chip must feature reverse-current protection (e.g., CH340K) to prevent power backfeeding and PDPowerMiniV1 startup failure when the UART side is powered up first.

## Version History
**V1.0.0.0**
1. Initial version

**V1.0.0.1**
1. Added zero-point calibration for voltage and current to improve measurement accuracy.
2. Optimized discharge logic: When output is enabled, discharge is enabled if (output voltage − set voltage) > 0.1V; if discharge lasts more than 5 seconds, it switches to 1‑second interval discharge mode. When output is disabled, discharge is enabled if output voltage > 0.1V; if discharge lasts more than 5 seconds, discharge is disabled.

**V1.0.0.2**
1. Added overcurrent discharge configuration, which supports disabling the overcurrent discharge function. This function is used to suppress output voltage overshoot under the "constant current sudden unload" scenario.
2. Adjusted the minimum set voltage and current to 0.5V and 0.005A. Accuracy is not guaranteed below 1V and below 50mA.

**V1.0.0.3**
1. Added configurable moving average filter window size for output voltage and current, allowing flexible adjustment of filtering effect.
2. Fixed the PD power display issue where the power showed "PD65WW" when switching from 100W to 65W.

**V1.0.0.4**
1. Optimize PD protocol, support EPR AVS, with the maximum requested voltage up to 24V.
2. Add serial number display on the boot screen.

**V1.0.0.5**
1. Add output display error compensation function (Output Comp):  
Voltage compensation range: ±0.02 V; Current compensation range: ±0.003 A.
2. Add UART communication function (UART Mode + UART Baudrate):  
`UART Mode`: 0 = Disable, 1 = Enable. Setting takes effect after power cycle.
`UART Baudrate`: Values 0-6 correspond to 9600, 19200, 38400, 57600, 115200, 230400, 460800 respectively.  
UART pins are assigned to DP and DM of Type‑C connector:
DP → TX, DM → RX, 3.3 V logic level.  
`Note`: The external UART chip must feature reverse-current protection (e.g., CH340K) to prevent power backfeeding and PDPowerMiniV1 startup failure when the UART side is powered up first.

## How to Upgrade, Windows
1. Extract WeActStudio_Upgrade_Tool.7z
2. Run WeActStudio_Upgrade_Tool.exe
3. Connect the device using a data cable
4. Select the fpk firmware
5. Open the serial port
6. Click the "Send" button to start the upgrade

## How to Upgrade, Linux , macOS or Windows
1. Extract WeActStudio_Upgrade_Tool_Python.zip
2. Connect the device using a data cable
3. Run WeActStudio_Upgrade_Tool.py, need to install pyserial library  
Example: python WeActStudio_Upgrade_Tool.py firmware.fpk
4. Wait for the upgrade to complete.