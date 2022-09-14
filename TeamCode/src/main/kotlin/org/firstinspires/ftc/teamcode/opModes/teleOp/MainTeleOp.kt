package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER
import com.arcrobotics.ftclib.gamepad.TriggerReader
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.IntakeCommand
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem

@TeleOp
class MainTeleOp: CommandOpMode() {
    private val intakeMotor = Motor(hardwareMap, "intake")

    private val intakeSubsystem = IntakeSubsystem(intakeMotor)

    private val intakeCommand = IntakeCommand(intakeSubsystem, true)
    private val outtakeCommand = IntakeCommand(intakeSubsystem, false)

    private val driver = GamepadEx(gamepad1)

    override fun initialize() {
        driver.getGamepadButton(A).whenHeld(intakeCommand)
        driver.getGamepadButton(B).whenHeld(outtakeCommand)
        schedule(
            ConditionalCommand(
                intakeCommand,
                outtakeCommand,
                TriggerReader(driver, LEFT_TRIGGER)::isDown
            )
        )
    }
}
