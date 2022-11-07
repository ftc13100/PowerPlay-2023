package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup

class OpenElevatorSubsystem(leftMotor: Motor, rightMotor: Motor): SubsystemBase() {
    private val elevatorMotors = MotorGroup(leftMotor, rightMotor)

    fun spinUp() = elevatorMotors.set(0.6)

    fun spinDown() = elevatorMotors.set(-0.6)

    fun stopSpin() = elevatorMotors.stopMotor()
}