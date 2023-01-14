package org.firstinspires.ftc.teamcode.opModes.teleOp.tests.slides

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem

@TeleOp
@Config
@Disabled
class SlidesPIDTuner : CommandOpMode() {
    //hardware
    private lateinit var leftMotor: Motor
    private lateinit var rightMotor: Motor
    private lateinit var limit: TouchSensor

    //subsystems
    private lateinit var slidesSubsystem: SlidesSubsystem

    //gamepads
    override fun initialize() {
        val dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(dashboard.telemetry, telemetry)

        leftMotor = Motor(hardwareMap, DeviceConfig.SLIDES_LEFT.deviceName)
        rightMotor = Motor(hardwareMap, DeviceConfig.SLIDES_RIGHT.deviceName)
        limit = hardwareMap.get(TouchSensor::class.java, DeviceConfig.SLIDES_LIMIT.deviceName)

        slidesSubsystem = SlidesSubsystem(leftMotor, rightMotor, limit, telemetry)

        register(slidesSubsystem)

        slidesSubsystem.defaultCommand =
            RunCommand(slidesSubsystem::operateSlides, slidesSubsystem)
    }
}