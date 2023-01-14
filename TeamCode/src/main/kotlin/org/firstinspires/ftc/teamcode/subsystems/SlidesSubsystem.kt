package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.constants.SlidesConst
import kotlin.math.sign

@Config
class SlidesSubsystem(
    slidesLeft: Motor,
    slidesRight: Motor,
    private val limit: TouchSensor,
    private val telemetry: Telemetry
) : SubsystemBase() {

    companion object {
        @JvmField
        var p: Double = 0.0

        @JvmField
        var i: Double = 0.0

        @JvmField
        var d: Double = 0.0

        @JvmField
        var target: Double = 3200.0
    }

    // Hardware
    private val slidesMotors = MotorGroup(slidesLeft, slidesRight)

    // Controllers
    private val controller = ProfiledPIDController(
        SlidesConst.SlidesPID.P.coeff,
        SlidesConst.SlidesPID.I.coeff,
        SlidesConst.SlidesPID.D.coeff,
        TrapezoidProfile.Constraints(
            SlidesConst.SlidesProfile.V.coeff,
            SlidesConst.SlidesProfile.A.coeff
        )
    )

    // Initialization
    init {
        slidesRight.inverted = true
        slidesMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        slidesMotors.resetEncoder()
        controller.setGoal(target)
    }

    // Methods
    fun setGoal(position: SlidesConst.SlidesPosition) = controller.setGoal(/*position.ticks*/ target)

    fun increaseTargetPosition(increase: Double) = controller.setGoal(controller.goal.position + increase)

    fun atGoal() = controller.atGoal()

    fun operateSlides() {
        controller.setPID(p, i, d)
        val basePower = controller.calculate(slidesMotors.positions.first())
        val error = basePower + sign(basePower) * SlidesConst.SlidesProfile.S.coeff  + SlidesConst.SlidesPID.G.coeff

        telemetry.addData("Current Position", slidesMotors.positions.first())
        telemetry.addData("Target Position", controller.setpoint.position)
        telemetry.addData("Goal Position", controller.goal.position)
        telemetry.addData("Motor Power", error)
        telemetry.update()

        slidesMotors.set(error)
    }

    fun stop() {
        if(controller.goal.position < 20) {
            slidesMotors.stopMotor()
        } else {
            slidesMotors.set(SlidesConst.SlidesPID.G.coeff)
        }
    }

    fun isPressed() = limit.isPressed

    // For tuning purposes ONLY
    fun spin() = slidesMotors.set(1.0)

    fun getVelocity(): Double = slidesMotors.velocities.first()

    fun setPower(pow: Double) =
        if (sign(pow) == -1.0 && isPressed()) {
            slidesMotors.stopMotor()
        } else {
            slidesMotors.set(pow)
        }
}