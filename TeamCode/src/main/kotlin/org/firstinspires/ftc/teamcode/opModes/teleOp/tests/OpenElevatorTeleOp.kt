package org.firstinspires.ftc.teamcode.opModes.teleOp.tests

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.commands.ElevatorSpinDownCommand
import org.firstinspires.ftc.teamcode.commands.ElevatorSpinUpCommand
import org.firstinspires.ftc.teamcode.constants.DeviceConfig.*
import org.firstinspires.ftc.teamcode.subsystems.OpenElevatorSubsystem

@TeleOp(name = "Open Loop Elevator Test")
class OpenElevatorTeleOp(): CommandOpMode() {
    override fun initialize() {
        val leftMotor = Motor(hardwareMap, SLIDES_LEFT.deviceName)
        val rightMotor = Motor(hardwareMap, SLIDES_RIGHT.deviceName)
        val limit = hardwareMap.get(TouchSensor::class.java, SLIDES_LIMIT.deviceName)
        leftMotor.inverted = true

        val subsystem = OpenElevatorSubsystem(leftMotor, rightMotor, limit, telemetry)

        val spinUpCommand = ElevatorSpinUpCommand(subsystem)
        val spinDownCommand = ElevatorSpinDownCommand(subsystem)

        val driver = GamepadEx(gamepad1)

        driver.getGamepadButton(RIGHT_BUMPER).whenHeld(spinUpCommand)
        driver.getGamepadButton(LEFT_BUMPER).whenHeld(spinDownCommand)
    }
}