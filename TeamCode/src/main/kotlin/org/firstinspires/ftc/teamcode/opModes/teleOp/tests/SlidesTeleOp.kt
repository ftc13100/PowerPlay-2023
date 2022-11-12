package org.firstinspires.ftc.teamcode.opModes.teleOp.tests

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.commands.SlidesCommand
import org.firstinspires.ftc.teamcode.constants.DeviceConfig.SLIDES_LEFT
import org.firstinspires.ftc.teamcode.constants.DeviceConfig.SLIDES_RIGHT
import org.firstinspires.ftc.teamcode.constants.DeviceConfig.SLIDES_LIMIT
import org.firstinspires.ftc.teamcode.constants.SlidesConst
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem

@TeleOp(name = "Slides Test", group = "Test")
class SlidesTeleOp(): CommandOpMode() {
    private val slidesLeft = Motor(hardwareMap, SLIDES_LEFT.deviceName)
    private val slidesRight = Motor(hardwareMap, SLIDES_RIGHT.deviceName)

    private val limit = hardwareMap.get(TouchSensor::class.java, SLIDES_LIMIT.deviceName)

    private val slidesSubsystem = SlidesSubsystem(slidesLeft, slidesRight, limit, telemetry)

    private val slidesGroundCommand = SlidesCommand(slidesSubsystem, SlidesConst.SlidesPosition.GROUND)
    private val slidesLowCommand = SlidesCommand(slidesSubsystem, SlidesConst.SlidesPosition.LOW)
    private val slidesMidCommand = SlidesCommand(slidesSubsystem, SlidesConst.SlidesPosition.MIDDLE)
    private val slidesHighCommand = SlidesCommand(slidesSubsystem, SlidesConst.SlidesPosition.HIGH)

    private val driver = GamepadEx(gamepad1)

    override fun initialize() {
        driver.getGamepadButton(DPAD_DOWN).whenPressed(slidesGroundCommand)
        driver.getGamepadButton(DPAD_LEFT).whenPressed(slidesLowCommand)
        driver.getGamepadButton(DPAD_RIGHT).whenPressed(slidesMidCommand)
        driver.getGamepadButton(DPAD_UP).whenPressed(slidesHighCommand)
    }
}