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
import org.firstinspires.ftc.teamcode.commands.claw.RotateClawCommand
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.slides.HeightCommand
import org.firstinspires.ftc.teamcode.commands.slides.SlidesCommand
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.constants.PoseStorage
import org.firstinspires.ftc.teamcode.constants.SlidesConst
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SlidesClawSubsystem
import org.firstinspires.ftc.teamcode.triggers.ClawSensorTrigger
import org.firstinspires.ftc.teamcode.triggers.JoystickTrigger

@TeleOp(name = "Main")
class MainTeleOp : CommandOpMode() {
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

        slidesGroundCommand = SlidesCommand(slidesClawSubsystem, SlidesConst.SlidesPosition.GROUND)
        slidesIntakeCommand = SlidesCommand(slidesClawSubsystem, SlidesConst.SlidesPosition.INTAKE)
        slidesLowCommand = SlidesCommand(slidesClawSubsystem, SlidesConst.SlidesPosition.LOW)
        slidesMidCommand = SlidesCommand(slidesClawSubsystem, SlidesConst.SlidesPosition.MIDDLE)
        slidesHighCommand = SlidesCommand(slidesClawSubsystem, SlidesConst.SlidesPosition.HIGH)

        rotateLeftCommand = RotateClawCommand(slidesClawSubsystem, SlidesConst.ClawPositions.LEFT, driveSubsystem::poseEstimate)
        rotateMidCommand = RotateClawCommand(slidesClawSubsystem, SlidesConst.ClawPositions.MIDDLE, driveSubsystem::poseEstimate)
        rotateRightCommand = RotateClawCommand(slidesClawSubsystem, SlidesConst.ClawPositions.RIGHT, driveSubsystem::poseEstimate)

        openClawCommand = InstantCommand({
            slidesClawSubsystem.openClaw()
            slidesClawSubsystem.clawState = SlidesConst.ClawState.OPEN
        }, slidesClawSubsystem)

        closeClawCommand = InstantCommand({
            slidesClawSubsystem.closeClaw()
            slidesClawSubsystem.clawState = SlidesConst.ClawState.CLOSE
        }, slidesClawSubsystem)

        adjustHeightCommand = HeightCommand(slidesClawSubsystem, operator::getLeftY)

        // Custom Triggers
        joystickTrigger = JoystickTrigger(operator::getLeftY)
        clawTrigger = ClawSensorTrigger(colorSensor, telemetry)

        driveSubsystem.poseEstimate = PoseStorage.poseEstimate

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
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            ConditionalCommand(
                openClawCommand,
                closeClawCommand,
                slidesClawSubsystem::isClosed
            )
        )

        joystickTrigger.whenActive(adjustHeightCommand)
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