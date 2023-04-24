package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.slides.SpinCommand
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.constants.PoseStorage
import org.firstinspires.ftc.teamcode.constants.SlidesConst
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SlidesClawSubsystem

@TeleOp(name = "Main (Outreach & One Controller)")
class OutreachTeleOp: CommandOpMode() {
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

    // Commands
    private lateinit var driveCommand: DriveCommand

    private lateinit var spinUpCommand: SpinCommand
    private lateinit var spinDownCommand: SpinCommand

    private lateinit var openClawCommand: InstantCommand
    private lateinit var closeClawCommand: InstantCommand


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
        driveSubsystem = DriveSubsystem(SampleMecanumDrive(hardwareMap), false)
        slidesClawSubsystem = SlidesClawSubsystem(slidesLeft, slidesRight, clawServo, rotationServo, limit, telemetry)

        // Gamepads
        driver = GamepadEx(gamepad1)

        // Commands
        driveCommand = DriveCommand(driveSubsystem,
            leftX = driver::getLeftX,
            leftY = driver::getLeftY,
            rightX = driver::getRightX,
            zoneVal = 0.15
        )

        spinUpCommand = SpinCommand(slidesClawSubsystem, true)
        spinDownCommand = SpinCommand(slidesClawSubsystem, false)

        openClawCommand = InstantCommand({
            slidesClawSubsystem.openClaw()
            slidesClawSubsystem.clawState = SlidesConst.ClawState.OPEN
        }, slidesClawSubsystem)

        closeClawCommand = InstantCommand({
            slidesClawSubsystem.closeClaw()
            slidesClawSubsystem.clawState = SlidesConst.ClawState.CLOSE
        }, slidesClawSubsystem)

        driveSubsystem.poseEstimate = PoseStorage.poseEstimate

        // Assign commands to gamepads
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(spinDownCommand)
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(spinUpCommand)

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            ConditionalCommand(
                openClawCommand,
                closeClawCommand,
                slidesClawSubsystem::isClosed
            )
        )

        // Register Subsystems
        register(driveSubsystem, slidesClawSubsystem)

        // Assign Default Commands
        driveSubsystem.defaultCommand = driveCommand
    }
}