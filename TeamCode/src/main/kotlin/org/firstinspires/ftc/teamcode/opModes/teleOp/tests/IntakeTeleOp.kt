package org.firstinspires.ftc.teamcode.opModes.teleOp.tests

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(group = "Test")
class IntakeTeleOp : CommandOpMode() {
    override fun initialize() {
        val hex1 = Motor(hardwareMap, "hex1")
        val driver = GamepadEx(gamepad1)

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
            InstantCommand({
                hex1.set(1.0)
            })
        ).whenReleased(
            InstantCommand({
                hex1.set(0.0)
            })
        )

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            InstantCommand({
                hex1.set(-1.0)
            })
        ).whenReleased(
            InstantCommand({
                hex1.set(0.0)
            })
        )
    }
}