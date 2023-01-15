package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.apache.commons.math3.geometry.euclidean.threed.FieldRotation
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.constants.SlidesConst
import kotlin.math.sign

@Config
class SlidesClawSubsystem(
    slidesLeft: Motor,
    slidesRight: Motor,
    private val clawServo: Servo,
    private val rotationServo: Servo,
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
            SlidesConst.SlidesConstraints.MAX_VELOCITY.value,
            SlidesConst.SlidesConstraints.MAX_ACCELERATION.value
        )
    )

    // Initialization
    init {
        slidesRight.inverted = true
        rotateMid()
        slidesMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        slidesMotors.resetEncoder()
        controller.setGoal(SlidesConst.SlidesPosition.GROUND.ticks)
        openClaw()
    }

    var goal: SlidesConst.SlidesPosition = SlidesConst.SlidesPosition.GROUND
        set(targetPos) {
            controller.setGoal(targetPos.ticks)
            field = targetPos
        }

    // Methods
    fun increaseTargetPosition(increase: Double) = controller.setGoal(controller.goal.position + increase)

    fun atGoal() = controller.atGoal()
    fun operateSlides() {
        val basePower = controller.calculate(slidesMotors.positions.first())
        val error = basePower + sign(basePower) * SlidesConst.SlidesProfile.S.coeff

        telemetry.addData("Current Position", slidesMotors.positions.first())
        telemetry.addData("Target Position", controller.setpoint.position)
        telemetry.addData("Goal Position", controller.goal.position)
        telemetry.addData("Motor Power", error)
        telemetry.update()

        slidesMotors.set(error)
    }

    fun stop() {
        slidesMotors.stopMotor()
    }

    fun isPressed() = limit.isPressed

    fun getVelocity(): Double = slidesMotors.velocities.first()

    fun setPower(pow: Double) =
        if (sign(pow) == -1.0 && isPressed()) {
            slidesMotors.stopMotor()
        } else {
            slidesMotors.set(pow)
        }

    fun openClaw() { clawServo.position = 1.0 }

    fun closeClaw() { clawServo.position = 0.0 }

    fun rotateLeft() { rotationServo.position = 1.0 }

    fun rotateNormal() { rotationServo.position = 0.4 }
    fun rotateMid() { rotationServo.position = 0.5 }

    fun rotateRight() { rotationServo.position = 0.0 }
}