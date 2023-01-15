package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.slides.HeightCommand
import org.firstinspires.ftc.teamcode.commands.slides.RotateClawCommand
import org.firstinspires.ftc.teamcode.commands.slides.SlidesCommand
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.constants.SlidesConst
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SlidesClawSubsystem
import org.firstinspires.ftc.teamcode.triggers.JoystickTrigger

@TeleOp(name = "Main")
class MainTeleOp : CommandOpMode() {
    // Hardware
    private lateinit var slidesLeft: Motor
    private lateinit var slidesRight: Motor
    private lateinit var limit: TouchSensor
    private lateinit var clawServo: Servo
    private lateinit var rotationServo: Servo

    // Subsystems
    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var slidesClawSubsystem: SlidesClawSubsystem

    // Gamepads
    private lateinit var driver: GamepadEx
    private lateinit var operator: GamepadEx

    // Commands
    private lateinit var driveCommand: DriveCommand

    private lateinit var slidesGroundCommand: SlidesCommand
    private lateinit var slidesIntakeCommand: SlidesCommand
    private lateinit var slidesLowCommand: SlidesCommand
    private lateinit var slidesMidCommand: SlidesCommand
    private lateinit var slidesHighCommand: SlidesCommand
    private lateinit var adjustHeightCommand: HeightCommand

    private lateinit var rotateMidCommand: RotateClawCommand
    private lateinit var rotateLeftCommand: RotateClawCommand
    private lateinit var rotateRightCommand: RotateClawCommand

    private lateinit var openClawCommand: InstantCommand
    private lateinit var closeClawCommand: InstantCommand
    // Custom Triggers
    private lateinit var joystickTrigger: JoystickTrigger

    override fun initialize() {
        // Debug
        telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)

        // Hardware
        slidesLeft = Motor(hardwareMap, DeviceConfig.SLIDES_LEFT.deviceName)
        slidesRight = Motor(hardwareMap, DeviceConfig.SLIDES_RIGHT.deviceName)
        clawServo = hardwareMap.get(Servo::class.java, DeviceConfig.CLAW_SERVO.deviceName)
        rotationServo = hardwareMap.get(Servo::class.java, DeviceConfig.ROTATION_SERVO.deviceName)
        limit = hardwareMap.get(TouchSensor::class.java, DeviceConfig.SLIDES_LIMIT.deviceName)

        // Subsystems
        driveSubsystem = DriveSubsystem(SampleMecanumDrive(hardwareMap), true)
        slidesClawSubsystem = SlidesClawSubsystem(slidesLeft, slidesRight, clawServo, rotationServo, limit, telemetry)

        // Gamepads
        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

        // Commands
        driveCommand = DriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, 0.15)

        slidesGroundCommand = SlidesCommand(slidesClawSubsystem, SlidesConst.SlidesPosition.GROUND)
        slidesIntakeCommand = SlidesCommand(slidesClawSubsystem, SlidesConst.SlidesPosition.INTAKE)
        slidesLowCommand = SlidesCommand(slidesClawSubsystem, SlidesConst.SlidesPosition.LOW)
        slidesMidCommand = SlidesCommand(slidesClawSubsystem, SlidesConst.SlidesPosition.MIDDLE)
        slidesHighCommand = SlidesCommand(slidesClawSubsystem, SlidesConst.SlidesPosition.HIGH)

        rotateLeftCommand = RotateClawCommand(slidesClawSubsystem, SlidesConst.ClawPositions.LEFT)
        rotateMidCommand = RotateClawCommand(slidesClawSubsystem, SlidesConst.ClawPositions.MIDDLE)
        rotateRightCommand = RotateClawCommand(slidesClawSubsystem, SlidesConst.ClawPositions.RIGHT)

        openClawCommand = InstantCommand({ slidesClawSubsystem.openClaw() }, slidesClawSubsystem)
        closeClawCommand = InstantCommand( { slidesClawSubsystem.closeClaw() }, slidesClawSubsystem)

        adjustHeightCommand = HeightCommand(slidesClawSubsystem, operator::getLeftY)

        // Custom Triggers
        joystickTrigger = JoystickTrigger(operator::getLeftY)

        // Assign commands to gamepads
        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(slidesGroundCommand)
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(slidesIntakeCommand)

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(slidesGroundCommand)
        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(slidesLowCommand)
        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(slidesMidCommand)
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(slidesHighCommand)

        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(rotateLeftCommand)
        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(rotateMidCommand)
        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(rotateRightCommand)
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(openClawCommand)
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(closeClawCommand)

        joystickTrigger.whenActive(adjustHeightCommand)

        // Register Subsystems
        register(driveSubsystem)

        // Assign Default Commands
        driveSubsystem.defaultCommand = driveCommand
    }
}