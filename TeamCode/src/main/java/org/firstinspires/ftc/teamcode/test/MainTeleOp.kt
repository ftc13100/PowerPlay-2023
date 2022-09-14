package org.firstinspires.ftc.teamcode.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B
import com.arcrobotics.ftclib.hardware.motors.Motor

class MainTeleOp: CommandOpMode() {
    private val intakeCommand = IntakeCommand(Motor(hardwareMap, "intake"), true)
    private val outtakeCommand = IntakeCommand(Motor(hardwareMap, "intake"), false)
    private val driver = GamepadEx(gamepad1)

    override fun initialize() {
        driver.getGamepadButton(A).whenHeld(intakeCommand)
        driver.getGamepadButton(B).whenHeld(outtakeCommand)
    }
}