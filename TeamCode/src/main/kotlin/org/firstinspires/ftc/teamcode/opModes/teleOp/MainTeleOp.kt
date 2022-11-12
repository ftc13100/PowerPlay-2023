package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.DriveCommand
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem

@TeleOp(name = "Main")
class MainTeleOp : CommandOpMode() {
    override fun initialize() {
        val driveSubsystem = DriveSubsystem(SampleMecanumDrive(hardwareMap), false)

        val driver = GamepadEx(gamepad1)

        val driveCommand = DriveCommand(
            drive = driveSubsystem,
            leftX = driver::getLeftX,
            leftY = driver::getLeftY,
            rightX = driver::getRightX,
            zoneVal = 0.3
        )

        register(driveSubsystem)
        driveSubsystem.defaultCommand = driveCommand
    }
}