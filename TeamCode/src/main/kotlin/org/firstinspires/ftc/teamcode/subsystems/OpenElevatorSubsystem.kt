package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.Telemetry

class OpenElevatorSubsystem(
    leftMotor: Motor,
    rightMotor: Motor,
    private val limit: TouchSensor,
    val telemetry: Telemetry
) : SubsystemBase() {
    private val elevatorMotors = MotorGroup(leftMotor, rightMotor)
    private var maxVel = 0.0

    init {
        elevatorMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
    }

    fun spinUp() {
        elevatorMotors.set(1.0)
        val current = elevatorMotors.velocities.first()
        maxVel = current.coerceAtLeast(maxVel)
        telemetry.addData("Velocity", current)
        telemetry.update()

    }

    fun getPosition(): Double = elevatorMotors.positions.first()

    fun spinDown() = elevatorMotors.set(-1.0)

    fun isPressed(): Boolean {
        return limit.isPressed
    }

    fun stopSpin() = elevatorMotors.stopMotor()
}