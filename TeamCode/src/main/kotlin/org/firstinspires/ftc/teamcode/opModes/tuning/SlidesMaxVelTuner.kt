package org.firstinspires.ftc.teamcode.opModes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem

@Config
@Autonomous(name = "MaxVel", group = "tuning")
class SlidesMaxVelTuner : LinearOpMode() {
    private var maxVelocity = 0.0

    companion object {
        @JvmField
        var RUNTIME = 2.0
    }

    override fun runOpMode() {
        val slidesLeft = Motor(hardwareMap, "slidesLeft")
        val slidesRight = Motor(hardwareMap, "slidesRight")

        val subsystem = SlidesSubsystem(slidesLeft, slidesRight)

        telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)

        telemetry.addLine("Slides will run at full speed for $RUNTIME seconds.")
        telemetry.addLine("")
        telemetry.addLine("Press start when ready.")
        telemetry.update()

        waitForStart()

        telemetry.clearAll()
        telemetry.update()

        subsystem.slideUp()
        val timer = ElapsedTime()

        while (!isStopRequested && timer.seconds() < Companion.RUNTIME) {
            val currentVelocity = subsystem.getVelocity()
            maxVelocity = currentVelocity.coerceAtLeast(maxVelocity)
        }

        telemetry.addData("Max Velocity", maxVelocity)
        telemetry.update()

        while (!isStopRequested && opModeIsActive()) idle()
    }
}