package org.firstinspires.ftc.teamcode.opModes.teleOp.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.PerpetualCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.commands.openElevator.ElevatorSpinDownCommand
import org.firstinspires.ftc.teamcode.commands.openElevator.ElevatorSpinUpCommand
import org.firstinspires.ftc.teamcode.constants.DeviceConfig.*
import org.firstinspires.ftc.teamcode.subsystems.OpenElevatorSubsystem
@Disabled
@TeleOp(name = "Open Loop Elevator Test")
class OpenElevatorTeleOp: CommandOpMode() {
    override fun initialize() {
        val dash = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        val leftMotor = Motor(hardwareMap, SLIDES_LEFT.deviceName)
        val rightMotor = Motor(hardwareMap, SLIDES_RIGHT.deviceName)
        val limit = hardwareMap.get(TouchSensor::class.java, SLIDES_LIMIT.deviceName)
        rightMotor.inverted = true

        val subsystem = OpenElevatorSubsystem(leftMotor, rightMotor, limit, telemetry)

        val spinUpCommand = ElevatorSpinUpCommand(subsystem)
        val spinDownCommand = ElevatorSpinDownCommand(subsystem)

        val driver = GamepadEx(gamepad1)

        driver.getGamepadButton(RIGHT_BUMPER).whileHeld(spinUpCommand).whenReleased(
            InstantCommand({
                subsystem.stopSpin()
            }, subsystem)
        )
        driver.getGamepadButton(LEFT_BUMPER).whileHeld(spinDownCommand).whenReleased(
            InstantCommand({
                subsystem.stopSpin()
            }, subsystem)
        )

        schedule(
            PerpetualCommand(
                InstantCommand({
                    telemetry.addData("Pos", subsystem.getPosition())
                    telemetry.update()
                })
            )
        )
    }
}