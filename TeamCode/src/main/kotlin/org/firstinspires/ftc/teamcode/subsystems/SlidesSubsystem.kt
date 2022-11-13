package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.constants.SlidesConst

@Config
class SlidesSubsystem(
    slidesLeft: Motor,
    slidesRight: Motor,
    private val limit: TouchSensor,
    private val telemetry: Telemetry
) : SubsystemBase() {
    //TODO Tune Kp, Ki, Kd, and max constraints

    companion object {
        @JvmField
        var kP: Double = 0.0

        @JvmField
        var kI: Double = 0.0

        @JvmField
        var kD: Double = 0.0

        @JvmField
        var kG: Double = 0.0
    }

    private val slidesMotors = MotorGroup(slidesLeft, slidesRight)

    private var targetPosition = SlidesConst.SlidesPosition.GROUND

    private val controller = com.arcrobotics.ftclib.controller.PIDController(
        SlidesConst.SlidesPID.P.coeff,
        SlidesConst.SlidesPID.I.coeff,
        SlidesConst.SlidesPID.D.coeff,
    )

    private val feedforward = ElevatorFeedforward (
        SlidesConst.SlidesFeedforward.Ks.coeff,
        SlidesConst.SlidesFeedforward.Kg.coeff,
        SlidesConst.SlidesFeedforward.Kv.coeff,
        SlidesConst.SlidesFeedforward.Ka.coeff,
    )

    fun operateSlides() {
        controller.setPID(kP, kI, kD)

        val error = controller.calculate(slidesMotors.positions.first()) + kG

        telemetry.addData("Current Position", slidesMotors.positions.first())

        slidesMotors.set(error)
    }

    fun slideUp() = slidesMotors.set(1.0)

    fun slideDown() = slidesMotors.set(-0.2)

    fun stop() = slidesMotors.stopMotor()

    fun atTargetPosition() = slidesMotors.atTargetPosition()

    fun getVelocity() = slidesMotors.velocity

    fun setTargetPosition(targetPosition: SlidesConst.SlidesPosition) {
        controller.setPoint = targetPosition.ticks
        this.targetPosition = targetPosition
    }

    fun isPressed() = limit.isPressed
}