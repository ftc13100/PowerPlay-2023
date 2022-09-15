package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.hardware.motors.Motor

class IntakeSubsystem(private val intake: Motor) {
    fun intake() = intake.set(1.0)

    fun outtake() = intake.set(-1.0)

    fun stop() = intake.stopMotor()

}