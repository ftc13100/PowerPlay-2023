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
    private val controller = com.arcrobotics.ftclib.controller.PIDController(
        SlidesConst.SlidesPID.P.coeff,
        SlidesConst.SlidesPID.I.coeff,
        SlidesConst.SlidesPID.D.coeff,
    )

    // Status
    private var targetPosition = SlidesConst.SlidesPosition.GROUND

    // Initialization
    init {
        slidesRight.inverted = true
        slidesMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        slidesMotors.resetEncoder()
        controller.setPoint = slidesMotors.positions.first()
    }

    // Periodic method of the subsystem, will get called before commands
    override fun periodic() {
        var error = 0.005
        if (targetPosition != SlidesConst.SlidesPosition.GROUND) {
            error = controller.calculate(slidesMotors.positions.first()) + SlidesConst.SlidesPID.G.coeff
        }

        telemetry.addData("Current Position", slidesMotors.positions.first())
        telemetry.addData("Target Position", targetPosition.ticks)
        telemetry.addData("Error", error)
        telemetry.update()

        slidesMotors.set(error)

        if (getTargetPosition() == SlidesConst.SlidesPosition.GROUND && atTargetPosition() || isPressed()) {
            stop()
        } else if (atTargetPosition()) {
            stall()
        }
    }

    // Methods
    fun setTargetPosition(value: SlidesConst.SlidesPosition){
        controller.setPoint = value.ticks
        targetPosition = value
    }

    fun getTargetPosition() = targetPosition

    fun increaseTargetPosition(increase: Double) { controller.setPoint += increase }

    fun atTargetPosition() = controller.atSetPoint()

    fun operateSlides() {
        var error = 0.005
        if (targetPosition != SlidesConst.SlidesPosition.GROUND) {
            error = controller.calculate(slidesMotors.positions.first()) + SlidesConst.SlidesPID.G.coeff
        }

        telemetry.addData("Current Position", slidesMotors.positions.first())
        telemetry.addData("Target Position", targetPosition.ticks)
        telemetry.addData("Error", error)
        telemetry.update()

        slidesMotors.set(error)
    }

    fun stall() = slidesMotors.set(SlidesConst.SlidesPID.G.coeff)

    fun stop() = slidesMotors.stopMotor()

    fun isPressed() = limit.isPressed
}