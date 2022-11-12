package org.firstinspires.ftc.teamcode.opModes.teleOp.tests

import com.arcrobotics.ftclib.command.CommandOpMode
<<<<<<< HEAD
import com.arcrobotics.ftclib.command.ConditionalCommand
=======
>>>>>>> feature/roadrunner
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.DriveCommand
<<<<<<< HEAD
import org.firstinspires.ftc.teamcode.commands.IntakeCommand
import org.firstinspires.ftc.teamcode.constants.DeviceConfig.INTAKE
=======
>>>>>>> feature/roadrunner
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem

@TeleOp(name = "Drive w/ Intake ", group = "Test")
class DriveTeleOp : CommandOpMode() {
    override fun initialize() {
<<<<<<< HEAD
        val intakeMotor = Motor(hardwareMap, INTAKE.deviceName)
=======
>>>>>>> feature/roadrunner

//        val intakeMotor = Motor(hardwareMap, DeviceConfig.INTAKE_MOTOR.deviceName)

//        val intakeSubsystem = IntakeSubsystem(intakeMotor)
        val driveSubsystem = DriveSubsystem(SampleMecanumDrive(hardwareMap), false)

//        val intakeCommand = IntakeCommand(intakeSubsystem, true)
//        val outtakeCommand = IntakeCommand(intakeSubsystem, false)

        val driver = GamepadEx(gamepad1)

        val driveCommand = DriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, 0.15)

<<<<<<< HEAD
        driver.getGamepadButton(A).whenHeld(intakeCommand)
        driver.getGamepadButton(B).whenHeld(outtakeCommand)

=======
>>>>>>> feature/roadrunner
        register(driveSubsystem)

        schedule(
            driveCommand,
        )
    }
}
