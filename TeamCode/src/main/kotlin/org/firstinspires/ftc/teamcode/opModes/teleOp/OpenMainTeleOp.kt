package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.*
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.slides.SpinCommand
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.constants.PoseStorage
import org.firstinspires.ftc.teamcode.constants.SlidesConst
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SlidesClawSubsystem
import org.firstinspires.ftc.teamcode.triggers.ClawSensorTrigger

@TeleOp(name = "Main (Open Loop)")
class OpenMainTeleOp : CommandOpMode() {
    // Hardware
    private lateinit var slidesLeft: Motor
    private lateinit var slidesRight: Motor
    private lateinit var limit: TouchSensor
    private lateinit var colorSensor: RevColorSensorV3
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

    private lateinit var spinUpCommand: SpinCommand
    private lateinit var spinDownCommand: SpinCommand

    private lateinit var openClawCommand: InstantCommand
    private lateinit var closeClawCommand: InstantCommand

    private lateinit var clawTrigger: ClawSensorTrigger

    override fun initialize() {
        // Debug
        telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)

        // Hardware
        slidesLeft = Motor(hardwareMap, DeviceConfig.SLIDES_LEFT.deviceName)
        slidesRight = Motor(hardwareMap, DeviceConfig.SLIDES_RIGHT.deviceName)
        clawServo = hardwareMap.get(Servo::class.java, DeviceConfig.CLAW_SERVO.deviceName)
        rotationServo = hardwareMap.get(Servo::class.java, DeviceConfig.ROTATION_SERVO.deviceName)
        limit = hardwareMap.get(TouchSensor::class.java, DeviceConfig.SLIDES_LIMIT.deviceName)
        colorSensor = hardwareMap.get(RevColorSensorV3::class.java, DeviceConfig.COLOR_SENSOR.deviceName)

        // Subsystems
        driveSubsystem = DriveSubsystem(SampleMecanumDrive(hardwareMap), false)
        slidesClawSubsystem = SlidesClawSubsystem(slidesLeft, slidesRight, clawServo, rotationServo, limit, telemetry)

        // Gamepads
        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

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

        // Custom Triggers
        clawTrigger = ClawSensorTrigger(colorSensor, telemetry)

        driveSubsystem.poseEstimate = PoseStorage.poseEstimate

        // Assign commands to gamepads
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(spinDownCommand)
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(spinUpCommand)

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            ConditionalCommand(
                openClawCommand,
                closeClawCommand,
                slidesClawSubsystem::isClosed
            )
        )

        clawTrigger.toggleWhenActive(
            InstantCommand({
                slidesClawSubsystem.closeClaw()
                slidesClawSubsystem.clawState = SlidesConst.ClawState.CLOSE
            }, slidesClawSubsystem)
        )

        // Register Subsystems
        register(driveSubsystem, slidesClawSubsystem)

        // Assign Default Commands
        driveSubsystem.defaultCommand = driveCommand

        schedule(
            PerpetualCommand(
                RunCommand({
                    telemetry.addData("Red: ", colorSensor.red())
                    telemetry.addData("Distance: ", colorSensor.getDistance(DistanceUnit.CM))
//                    telemetry.update()
                })
            ),
        )
    }
}