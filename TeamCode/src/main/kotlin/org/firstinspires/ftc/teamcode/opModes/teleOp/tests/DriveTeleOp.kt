package org.firstinspires.ftc.teamcode.opModes.teleOp.tests

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.Subsystem
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER
import com.arcrobotics.ftclib.gamepad.TriggerReader
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.DriveCommand
import org.firstinspires.ftc.teamcode.commands.IntakeCommand
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem

@TeleOp(name = "Drive w/ Intake ", group = "Test")
class DriveTeleOp : CommandOpMode() {
    override fun initialize() {
        val intakeMotor = Motor(hardwareMap, DeviceConfig.INTAKE_MOTOR.deviceName)

        val intakeSubsystem = IntakeSubsystem(intakeMotor)
        val driveSubsystem = DriveSubsystem(SampleMecanumDrive(hardwareMap), false)

        val intakeCommand = IntakeCommand(intakeSubsystem, true)
        val outtakeCommand = IntakeCommand(intakeSubsystem, false)

        val driver = GamepadEx(gamepad1)

        val driveCommand = DriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, 0.15)

        driver.getGamepadButton(A).whenHeld(intakeCommand)
        driver.getGamepadButton(B).whenHeld(outtakeCommand)

        register(driveSubsystem as Subsystem)

        schedule(
            ConditionalCommand(intakeCommand, outtakeCommand, TriggerReader(driver, LEFT_TRIGGER)::isDown),
            driveCommand,
        )
    }
}
