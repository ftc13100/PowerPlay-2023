package org.firstinspires.ftc.teamcode.triggers

import com.arcrobotics.ftclib.command.button.Trigger
import java.util.function.DoubleSupplier

class JoystickTrigger(private val stickY: DoubleSupplier) : Trigger() {
    override fun get(): Boolean = stickY.asDouble != 0.0
}