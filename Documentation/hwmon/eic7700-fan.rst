.. SPDX-License-Identifier: GPL-2.0

Kernel driver eic7700-fan
=========================

The ESWIN EIC7700 fan controller counts APB clock cycles between
tachometer pulses. The driver exposes the speed in RPM through
``fan1_input`` on the ``eic7700_fan`` hwmon device. A locked fan or a
measurement that receives no pulses within 1.5 seconds reads as 0 RPM.

On boards with a PWM-controlled fan, the separate :doc:`pwm-fan` driver
provides ``pwm1`` on the ``pwmfan`` hwmon device. Supply a device tree
with a ``pwm-fan`` node to enable PWM control.
