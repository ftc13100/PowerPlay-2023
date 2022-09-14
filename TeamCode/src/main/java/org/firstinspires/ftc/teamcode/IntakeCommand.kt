package org.firstinspires.ftc.teamcode

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.hardware.motors.Motor

class IntakeCommand(private val intake: Motor) : CommandBase() {
    override fun execute() {
        intake.set(1.0)
    }

    override fun end(interrupted: Boolean) {
        intake.set(0.0)
    }
}