package org.firstinspires.ftc.teamcode.opModes.teleOp.tests.slides

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem

@Autonomous
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

        val subsystem = SlidesSubsystem(slidesLeft, slidesRight, limit, telemetry)
        telemetry = MultipleTelemetry(dashboard.telemetry, telemetry)

        waitForStart()

        timer.reset()
        while (opModeIsActive() and (timer.seconds() < TIME_TO_RUN)){
            subsystem.spin()

            val velocity = subsystem.getVelocity()

            if (velocity > MAX_VELOCITY){
                MAX_VELOCITY = velocity
            }

            telemetry.addData("Max Velocity", MAX_VELOCITY)
            telemetry.update()

            if(isStopRequested) break
        }

        telemetry.addData("Final Max Velocity", MAX_VELOCITY)
        telemetry.update()
    }
}