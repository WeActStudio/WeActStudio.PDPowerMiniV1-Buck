> After upgrading, please manually restore the factory settings to ensure the device can continue to operate abnormally.

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