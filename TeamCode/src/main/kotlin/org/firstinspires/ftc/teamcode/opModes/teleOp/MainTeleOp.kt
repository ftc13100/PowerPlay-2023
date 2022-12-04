package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.commands.DriveCommand
import org.firstinspires.ftc.teamcode.commands.IntakeCommand
import org.firstinspires.ftc.teamcode.commands.SlidesCommand
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.constants.SlidesConst
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem

@TeleOp(name = "Main")
class MainTeleOp : CommandOpMode() {
    // Hardware
    private lateinit var slidesLeft: Motor
    private lateinit var slidesRight: Motor
    private lateinit var limit: TouchSensor

    private lateinit var intakeMotor: Motor

    // Subsystems
    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var slidesSubsystem: SlidesSubsystem
    private lateinit var intakeSubsystem: IntakeSubsystem

    // Commands
    private lateinit var driveCommand: DriveCommand

    private lateinit var slidesGroundCommand: SlidesCommand
    private lateinit var slidesIntakeCommand: SlidesCommand
    private lateinit var slidesLowCommand: SlidesCommand
    private lateinit var slidesMidCommand: SlidesCommand
    private lateinit var slidesHighCommand: SlidesCommand

    private lateinit var intakeCommand: IntakeCommand
    private lateinit var outtakeCommand: IntakeCommand

    // Gamepads
    private lateinit var driver: GamepadEx
    private lateinit var operator: GamepadEx

    override fun initialize() {
        // Debug
        telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)

        // Hardware
        slidesLeft = Motor(hardwareMap, DeviceConfig.SLIDES_LEFT.deviceName)
        slidesRight = Motor(hardwareMap, DeviceConfig.SLIDES_RIGHT.deviceName)
        limit = hardwareMap.get(TouchSensor::class.java, DeviceConfig.SLIDES_LIMIT.deviceName)

        intakeMotor = Motor(hardwareMap, DeviceConfig.INTAKE.deviceName)

        // Subsystems
        driveSubsystem = DriveSubsystem(SampleMecanumDrive(hardwareMap), false)
        slidesSubsystem = SlidesSubsystem(slidesLeft, slidesRight, limit, telemetry)
        intakeSubsystem = IntakeSubsystem(intakeMotor)

        // Gamepads
        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

        // Commands
        driveCommand = DriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, 0.15)

        slidesGroundCommand = SlidesCommand(slidesSubsystem, SlidesConst.SlidesPosition.GROUND)
        slidesIntakeCommand = SlidesCommand(slidesSubsystem, SlidesConst.SlidesPosition.INTAKE)
        slidesLowCommand = SlidesCommand(slidesSubsystem, SlidesConst.SlidesPosition.LOW)
        slidesMidCommand = SlidesCommand(slidesSubsystem, SlidesConst.SlidesPosition.MIDDLE)
        slidesHighCommand = SlidesCommand(slidesSubsystem, SlidesConst.SlidesPosition.HIGH)

        intakeCommand = IntakeCommand(intakeSubsystem, true)
        outtakeCommand = IntakeCommand(intakeSubsystem, false)

        // Assign commands to gamepads
        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(slidesGroundCommand)
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(slidesIntakeCommand)

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(intakeCommand)
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(intakeCommand)

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(outtakeCommand)
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(outtakeCommand)

        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(slidesGroundCommand)
        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(slidesLowCommand)
        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(slidesMidCommand)
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(slidesHighCommand)

        // Register Subsystems
        register(driveSubsystem)

        // Assign Default Commands
        driveSubsystem.defaultCommand = driveCommand
    }
}