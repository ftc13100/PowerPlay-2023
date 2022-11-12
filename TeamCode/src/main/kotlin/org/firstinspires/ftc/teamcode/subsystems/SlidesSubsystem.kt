package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup

class SlidesSubsystem(slidesLeft: Motor, slidesRight: Motor): SubsystemBase() {
    //TODO Tune Kp, Ki, Kd, and max constraints

    companion object {
        @JvmField
        var kP: Double = 0.0

        @JvmField
        var kI: Double = 0.0

        @JvmField
        var kD: Double = 0.0

        @JvmField
        var kF: Double = 0.0

        @JvmField
        var goal = 0.0
    }

    private val slidesMotors = MotorGroup(slidesLeft, slidesRight)

    private val controller = com.arcrobotics.ftclib.controller.PIDFController(
        0.0 /* P.coeff */,
        0.0 /* I.coeff */,
        0.0 /* D.coeff */,
        0.0
    )

    private val feedforward = ElevatorFeedforward(
        0.0 /* Ks.coeff */,
        0.0 /* Kg.coeff */,
        0.0 /* Kv.coeff */,
    )

    fun operateSlides() {
        controller.setPIDF(kP, kI, kD, kF)
        val error = controller.calculate(slidesMotors.positions.first())

        slidesMotors.set(error)
    }

    fun slideUp() = slidesMotors.set(1.0)

    fun slideDown() = slidesMotors.set(-0.2)

    fun stop() = slidesMotors.stopMotor()

    fun getVelocity() = slidesMotors.velocity
}