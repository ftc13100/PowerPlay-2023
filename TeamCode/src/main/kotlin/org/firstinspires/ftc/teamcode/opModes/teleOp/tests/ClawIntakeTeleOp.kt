package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.subsystems.ClawIntakeSubsystem
import org.firstinspires.ftc.teamcode.triggers.ClawSensorTrigger

@TeleOp(name = "Main")
class ClawIntakeTeleOp : CommandOpMode() {
    // Hardware
    private lateinit var clawServo: Servo
    private lateinit var rotationServo: Servo
    private lateinit var colorSensor: ColorSensor
    private lateinit var distanceSensor: DistanceSensor

    // Subsystems
    private lateinit var intakeSubsystem: ClawIntakeSubsystem

    // Gamepads
    private lateinit var driver: GamepadEx

    // Custom Triggers
    private lateinit var clawSensorTrigger: ClawSensorTrigger

    override fun initialize() {
        // Debug
        telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)

        // Hardware
        clawServo = hardwareMap.get(Servo::class.java, DeviceConfig.CLAW_SERVO.deviceName)
        rotationServo = hardwareMap.get(Servo::class.java, DeviceConfig.ROTATION_SERVO.deviceName)
        colorSensor = hardwareMap.get(ColorSensor::class.java, DeviceConfig.COLOR_SENSOR.deviceName)
        distanceSensor = hardwareMap.get(DistanceSensor::class.java, DeviceConfig.COLOR_SENSOR.deviceName)

        // Subsystems
        intakeSubsystem = ClawIntakeSubsystem(clawServo, rotationServo)

        // Gamepads
        driver = GamepadEx(gamepad1)

        // Custom Triggers
        clawSensorTrigger = ClawSensorTrigger(colorSensor, distanceSensor)

        // Assign commands to gamepads
        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
            InstantCommand(
                { intakeSubsystem.rotateLeft() },
                intakeSubsystem
            )
        )

        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
            InstantCommand(
                { intakeSubsystem.rotateMid() },
                intakeSubsystem
            )
        )

        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
            InstantCommand(
                { intakeSubsystem.rotateRight() },
                intakeSubsystem
            )
        )

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
            InstantCommand(
                { intakeSubsystem.openClaw() },
                intakeSubsystem
            )
        )

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            InstantCommand(
                { intakeSubsystem.closeClaw() },
                intakeSubsystem
            )
        )


        clawSensorTrigger.whenActive(
            InstantCommand(
                { intakeSubsystem.closeClaw() },
                intakeSubsystem
            )
        )
    }
}