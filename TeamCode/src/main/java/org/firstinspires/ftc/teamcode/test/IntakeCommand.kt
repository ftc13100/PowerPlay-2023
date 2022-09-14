package org.firstinspires.ftc.teamcode.test

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.hardware.motors.Motor

class IntakeCommand(private val motor: Motor, private val intake: Boolean) : CommandBase() {
    override fun execute() {
        motor.set(
            if(intake) 1.0
            else -1.0
        )
    }

    override fun end(interrupted: Boolean) {
        motor.set(0.0)
    }
}