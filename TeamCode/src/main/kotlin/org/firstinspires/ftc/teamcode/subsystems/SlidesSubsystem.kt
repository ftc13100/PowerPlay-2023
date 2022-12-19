package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.constants.SlidesConst

class SlidesSubsystem(
    slidesLeft: Motor,
    slidesRight: Motor,
    private val limit: TouchSensor,
    private val telemetry: Telemetry
) : SubsystemBase() {
    // Hardware
    private val slidesMotors = MotorGroup(slidesLeft, slidesRight)

    // Controllers
    private val controller = ProfiledPIDController(
        SlidesConst.SlidesPID.P.coeff,
        SlidesConst.SlidesPID.I.coeff,
        SlidesConst.SlidesPID.D.coeff,
        TrapezoidProfile.Constraints(
            SlidesConst.SlidesProfile.kV.coeff,
            SlidesConst.SlidesProfile.kA.coeff
        )
    )

    // Initialization
    init {
        slidesRight.inverted = true
        slidesMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        slidesMotors.resetEncoder()
        controller.setGoal(slidesMotors.positions.first())
    }

    // Methods
    fun setGoal(position: SlidesConst.SlidesPosition) = controller.setGoal(position.ticks)

    fun increaseTargetPosition(increase: Double) = controller.setGoal(controller.goal.position + increase)

    fun atGoal() = controller.atGoal()

    fun operateSlides() {
        var error = 0.005
        if (targetPosition != SlidesConst.SlidesPosition.GROUND) {
            error = controller.calculate(slidesMotors.positions.first()) + SlidesConst.SlidesPID.G.coeff
        }

        telemetry.addData("Current Position", slidesMotors.positions.first())
        telemetry.addData("Target Position", controller.setpoint.position)
        telemetry.addData("Goal Position", controller.goal.position)
        telemetry.addData("Motor Power", error)
        telemetry.update()

        slidesMotors.set(error)
    }

    fun stall() = slidesMotors.set(SlidesConst.SlidesPID.G.coeff)

    fun stop() = slidesMotors.stopMotor()

    fun isPressed() = limit.isPressed
}