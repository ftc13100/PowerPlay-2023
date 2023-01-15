package org.firstinspires.ftc.teamcode.opModes.config.slides

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.subsystems.SlidesClawSubsystem

@Autonomous(group = "Slides Tuning")
@Disabled
@Config
class SlidesMaxVelocityTuner: LinearOpMode() {
    companion object{
        @JvmField
        var MAX_VELOCITY: Double = 0.0

        @JvmField
        var TIME_TO_RUN: Double = 2.0
    }

    override fun runOpMode() {
        val dashboard = FtcDashboard.getInstance()
        val timer = ElapsedTime()

        val slidesLeft = Motor(hardwareMap, DeviceConfig.SLIDES_LEFT.deviceName)
        val slidesRight = Motor(hardwareMap, DeviceConfig.SLIDES_RIGHT.deviceName)
        val limit = hardwareMap.get(TouchSensor::class.java, DeviceConfig.SLIDES_LIMIT.deviceName)
        val clawServo = hardwareMap.get(Servo::class.java, DeviceConfig.CLAW_SERVO.deviceName)
        val rotationServo = hardwareMap.get(Servo::class.java, DeviceConfig.ROTATION_SERVO.deviceName)

        val subsystem = SlidesClawSubsystem(slidesLeft, slidesRight, clawServo, rotationServo, limit, telemetry)
        telemetry = MultipleTelemetry(dashboard.telemetry, telemetry)

        waitForStart()

        timer.reset()
        while (opModeIsActive() and (timer.seconds() < TIME_TO_RUN)){
            subsystem.setPower(1.0)

            MAX_VELOCITY = subsystem.getVelocity().coerceAtLeast(MAX_VELOCITY)

            telemetry.addData("Max Velocity", MAX_VELOCITY)
            telemetry.update()

            if(isStopRequested) break
        }

        telemetry.addData("Final Max Velocity", MAX_VELOCITY)
        telemetry.update()
    }
}