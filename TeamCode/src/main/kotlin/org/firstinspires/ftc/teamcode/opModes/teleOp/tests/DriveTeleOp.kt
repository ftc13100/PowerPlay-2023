package org.firstinspires.ftc.teamcode.opModes.teleOp.tests

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand
import org.firstinspires.ftc.teamcode.constants.DeviceConfig.INTAKE
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem

@Disabled
@TeleOp(name = "Drive w/ Intake ", group = "Test")
class DriveTeleOp : CommandOpMode() {
    override fun initialize() {
        val intakeMotor = MotorEx(hardwareMap, INTAKE.deviceName)

        val intakeSubsystem = IntakeSubsystem(intakeMotor)
        val driveSubsystem = DriveSubsystem(SampleMecanumDrive(hardwareMap), false)

        val intakeCommand = IntakeCommand(intakeSubsystem, true)
        val outtakeCommand = IntakeCommand(intakeSubsystem, false)

        val driver = GamepadEx(gamepad1)

        val driveCommand = DriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, 0.15)
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(intakeCommand)
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(outtakeCommand)

        register(driveSubsystem)

        schedule(
            driveCommand,
        )
    }
}
