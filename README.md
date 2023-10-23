# sixDigitNixieClock
Yet another Six digit (multiplexed) Nixie clock firmware. This one matches my own driver board.

The code is not clean but it got some optimization to get rid of flickering. The code is supposed to be compiled in Arduino IDE.

This firmware
- generates the PWM to boost the power supply voltage up to 170-200 V
- reads and sets the time from a DS3231 I2C RTC chip
- handles digit multiplexing


**NOTE** There is currently no feedback loop on the HVgenerator (there is the code resulting from copy&paste but it is not used). OTOH there is a calibration feature at bootstrap moment and the pwmTop value is retained in EEPROM (until next firmware flash).

**NOTE** There is no provision for LED backlight, neither in the code nor in my driver board. Look somewhere else for that feature (hint: I copied the HV generator code from his project).
