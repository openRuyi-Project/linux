.. SPDX-License-Identifier: GPL-2.0

Kernel driver eic7700-fan
=========================

Supported chips:

  * ESWIN EIC7700 SoC

    Prefix: 'eic7700_fan'

    Datasheet: EIC7700X SoC Technical Reference Manual, FAN_CTRL chapter

Description
-----------

The EIC7700 fan controller measures the period of the fan tachometer
signal in APB clock cycles. The fan speed is controlled through an
external PWM channel described by the ``pwms`` devicetree property.

If the devicetree provides ``cooling-levels``, the fan is also registered
as a thermal cooling device and each cooling state selects the
corresponding PWM duty cycle, so that thermal zones can control the fan.

The driver starts the fan at its highest level. A stalled fan, or one that
does not produce tachometer pulses within 1.5 seconds, is reported as 0 RPM.

Sysfs entries
-------------

=============== ======= =================================================
fan1_input      RO      Fan speed in RPM
pwm1            RW      PWM duty cycle, from 25 (slowest) to 255 (full speed)
=============== ======= =================================================
