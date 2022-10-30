package org.firstinspires.ftc.teamcode.opModes.teleOp.tests

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(group = "Test")
class IntakeTeleOp : CommandOpMode() {
    private var hex1: Motor? = null
    private var gamepad: GamepadEx? = null
    override fun initialize() {
        hex1 = Motor(hardwareMap, "hex1")
        gamepad = GamepadEx(gamepad1)

        gamepad!!.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
            InstantCommand({
                hex1!!.set(1.0)
            })
        ).whenReleased(
            InstantCommand({
                hex1!!.set(0.0)
            })
        )

        gamepad!!.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            InstantCommand({
                hex1!!.set(-1.0)
            })
        ).whenReleased(
            InstantCommand({
                hex1!!.set(0.0)
            })
        )
    }
}