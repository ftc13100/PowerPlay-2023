package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.TouchSensor

class OpenElevatorSubsystem(leftMotor: Motor, rightMotor: Motor, private val limit: TouchSensor): SubsystemBase() {
    private val elevatorMotors = MotorGroup(leftMotor, rightMotor)

    fun spinUp() = elevatorMotors.set(1.0)

    fun spinDown() = elevatorMotors.set(-1.0)

    fun isPressed() : Boolean { return limit.isPressed };

    fun stopSpin() = elevatorMotors.stopMotor()
}