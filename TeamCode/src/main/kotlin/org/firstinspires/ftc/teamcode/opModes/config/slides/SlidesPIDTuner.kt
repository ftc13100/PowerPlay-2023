package org.firstinspires.ftc.teamcode.opModes.config.slides

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.subsystems.SlidesClawSubsystem

@TeleOp
@Config
@Disabled
class SlidesPIDTuner : CommandOpMode() {

    //hardware
    private lateinit var leftMotor: Motor
    private lateinit var rightMotor: Motor
    private lateinit var limit: TouchSensor
    private lateinit var clawServo: Servo
    private lateinit var rotationServo: Servo

    //subsystems
    private lateinit var slidesClawSubsystem: SlidesClawSubsystem

    //gamepads
    override fun initialize() {
        val dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(dashboard.telemetry, telemetry)

        leftMotor = Motor(hardwareMap, DeviceConfig.SLIDES_LEFT.deviceName)
        rightMotor = Motor(hardwareMap, DeviceConfig.SLIDES_RIGHT.deviceName)
        limit = hardwareMap.get(TouchSensor::class.java, DeviceConfig.SLIDES_LIMIT.deviceName)
        clawServo = hardwareMap.get(Servo::class.java, DeviceConfig.CLAW_SERVO.deviceName)
        rotationServo = hardwareMap.get(Servo::class.java, DeviceConfig.ROTATION_SERVO.deviceName)

        slidesClawSubsystem = SlidesClawSubsystem(leftMotor, rightMotor, clawServo, rotationServo, limit, telemetry)

        register(slidesClawSubsystem)

        slidesClawSubsystem.defaultCommand =
            RunCommand(slidesClawSubsystem::operateSlides, slidesClawSubsystem)
    }
}