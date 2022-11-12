package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.TouchSensor

class SlidesSubsystem(slidesLeft: Motor, slidesRight: Motor, private val limit: TouchSensor): SubsystemBase() {
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
        var goal: Double = 0.0
    }

    private val slidesMotors = MotorGroup(slidesLeft, slidesRight)

    private val controller = PIDFController(
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
        controller.setPoint = goal
        controller.setPIDF(kP, kI, kD, kF)
        val error = controller.calculate(slidesMotors.positions.first())

        slidesMotors.set(error)
    }

    fun setGoal(newGoal: Double) {
        goal = newGoal
        controller.setPoint = goal
    }

    fun atGoal() : Boolean { return controller.atSetPoint() }

    fun limitReached() : Boolean { return limit.isPressed }

    fun stop() = slidesMotors.stopMotor()

    fun getVelocity() = slidesMotors.velocity
}