package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup

class SlidesSubsystem(slidesLeft: Motor, slidesRight: Motor): SubsystemBase() {
    private val slideMotors = MotorGroup(slidesLeft, slidesRight)

    fun slideUp() = slideMotors.set(0.2)

    fun slideDown() = slideMotors.set(-0.2)

    fun stop() = slideMotors.stopMotor()
}