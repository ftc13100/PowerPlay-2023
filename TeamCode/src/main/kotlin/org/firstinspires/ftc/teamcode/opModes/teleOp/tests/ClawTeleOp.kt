package org.firstinspires.ftc.teamcode.opModes.teleOp.tests

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.commands.claw.GrabCommand
import org.firstinspires.ftc.teamcode.subsystems.ClawIntakeSubsystem

@TeleOp(name = "Claw Test", group = "Tests")
class ClawTeleOp() : CommandOpMode() {
    private lateinit var clawServo: Servo
    private lateinit var rotationServo: Servo

    private lateinit var subsystem: ClawIntakeSubsystem

    private lateinit var grabCommand: GrabCommand
    private lateinit var releaseCommand: GrabCommand

    private lateinit var driver: GamepadEx

    override fun initialize() {
        clawServo = hardwareMap.get(Servo::class.java, "Claw Servo")
        rotationServo = hardwareMap.get(Servo::class.java, "Spin Servo")

        subsystem = ClawIntakeSubsystem(clawServo, rotationServo)

        grabCommand = GrabCommand(true, subsystem)
        releaseCommand = GrabCommand(false, subsystem)

        driver = GamepadEx(gamepad1)

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(grabCommand)
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(releaseCommand)
    }
}
