package org.firstinspires.ftc.teamcode.opModes.tuning

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem

@Config
@Autonomous(name = "MaxVel", group = "tuning")
class SlidesMaxVelTuner : LinearOpMode() {
    private val slidesLeft = Motor(hardwareMap, "slidesLeft")
    private val slidesRight = Motor(hardwareMap, "slidesRight")

    private val subsystem = SlidesSubsystem(slidesLeft, slidesRight)

    private val timer = ElapsedTime()

    @JvmField
    var RUNTIME = 2.0

    override fun runOpMode() {
        TODO("Not yet implemented")
    }
}