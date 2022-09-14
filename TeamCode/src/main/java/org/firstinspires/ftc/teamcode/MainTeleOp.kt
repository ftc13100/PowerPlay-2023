package org.firstinspires.ftc.teamcode

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor

class MainTeleOp: CommandOpMode() {
    private val intakeCommand = IntakeCommand(Motor(hardwareMap, "intake"))
    private val driver = GamepadEx(gamepad1)

    override fun initialize() {
        driver.getGamepadButton(GamepadKeys.Button.A).whenHeld(intakeCommand)
    }
}