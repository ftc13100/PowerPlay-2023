package org.firstinspires.ftc.teamcode.opModes.teleOp.tests

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.SlidesCommand
import org.firstinspires.ftc.teamcode.constants.DeviceConfig.SLIDES_LEFT
import org.firstinspires.ftc.teamcode.constants.DeviceConfig.SLIDES_RIGHT
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem

@TeleOp(name = "Slides Test ", group = "Test")
class SlidesTeleOp(): CommandOpMode() {
    private val slidesLeft = Motor(hardwareMap, SLIDES_LEFT.deviceName)
    private val slidesRight = Motor(hardwareMap, SLIDES_RIGHT.deviceName)

    private val slidesSubsystem = SlidesSubsystem(slidesLeft, slidesRight)

    private val slidesCommandUp = SlidesCommand(slidesSubsystem, true)
    private val slidesCommandDown = SlidesCommand(slidesSubsystem, false)

    private val driver = GamepadEx(gamepad1)

    override fun initialize() {
        driver.getGamepadButton(DPAD_UP).whenHeld(slidesCommandUp)
        driver.getGamepadButton(DPAD_DOWN).whenHeld(slidesCommandDown)
    }
}