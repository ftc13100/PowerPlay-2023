package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile

class SlidesSubsystem(slidesLeft: Motor, slidesRight: Motor): SubsystemBase() {
    private val slidesMotors = MotorGroup(slidesLeft, slidesRight)

    private val controller = ProfiledPIDController(
        0.0 /* P.coeff */,
        0.0 /* I.coeff */,
        0.0 /* D.coeff */,
        TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
    )
    private val feedforward = ElevatorFeedforward(
        0.0 /* Ks.coeff */,
        0.0 /* Kg.coeff */,
        0.0 /* Kv.coeff */,
    )

    fun operateSlides() {
        val ff = feedforward.calculate(slidesMotors.velocity)
        val error = controller.calculate(slidesMotors.positions.first())

        slidesMotors.set(error + ff)
    }

    fun slideUp() = slidesMotors.set(1.0)

    fun slideDown() = slidesMotors.set(-0.2)

    fun stop() = slidesMotors.stopMotor()

    fun getVelocity() = slidesMotors.velocity

    companion object {
        @JvmField
        var kP: Double = 0.0

        @JvmField
        var kI: Double = 0.0

        @JvmField
        var kD: Double = 0.0

        @JvmField
        var maxVelocity = 0.0

        @JvmField
        var maxAcceleration = 0.0

        @JvmField
        var goal = 0.0
    }
}