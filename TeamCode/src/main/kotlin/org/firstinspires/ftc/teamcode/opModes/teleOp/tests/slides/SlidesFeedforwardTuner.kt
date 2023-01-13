package org.firstinspires.ftc.teamcode.opModes.teleOp.tests.slides

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.util.NanoClock
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry
import org.firstinspires.ftc.teamcode.constants.SlidesConst
import org.firstinspires.ftc.teamcode.constants.SlidesConst.SlidesConstraints.*
import org.firstinspires.ftc.teamcode.opModes.teleOp.tests.slides.SlidesFeedforwardTuner.Companion.generateProfile
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kV
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem

class SlidesFeedforwardTuner(): LinearOpMode() {
    companion object {
        @JvmField
        val DISTANCE = 800.0 //encoder ticks

        @JvmStatic
        fun generateProfile(forward: Boolean): MotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                start = if (forward) MotionState(
                    0.0,
                    0.0
                ) else MotionState(
                    DISTANCE,
                    0.0
                ),
                goal = if (forward) MotionState(
                    DISTANCE,
                    0.0,
                    0.0
                ) else MotionState(
                    0.0,
                    0.0
                ),
                maxVel = MAX_VELOCITY.value,
                maxAccel = MAX_ACCELERATION.value
            )

        @JvmField
        val kS = 0.0

        @JvmField
        val kV = 0.0

        @JvmField
        val kA = 0.0
    }

    private lateinit var slidesLeft: Motor
    private lateinit var slidesRight: Motor
    private lateinit var limit: TouchSensor


    private lateinit var subsystem: SlidesSubsystem
    private val dashboard = FtcDashboard.getInstance()

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        subsystem = SlidesSubsystem(slidesLeft, slidesRight, limit, telemetry)

        val clock = NanoClock.system()

        telemetry.addLine("Ready!")
        telemetry.update()
        telemetry.clearAll()

        waitForStart()

        if(isStopRequested) return

        var forward = true
        var activeProfile = generateProfile(forward)
        var profileStart = clock.seconds()

        while(!isStopRequested) {
            val profileTime = clock.seconds() - profileStart

            if(profileTime > activeProfile.duration()) {
                forward = forward.apply { not() }
                activeProfile = generateProfile(forward)
                profileStart = clock.seconds()
            }

            val motionState = activeProfile[profileTime]
            val targetPower = Kinematics.calculateMotorFeedforward(motionState.v, motionState.a, kV, kA, kS)

            subsystem.setPower(targetPower)
            val currentVelo = subsystem.getVelocity()

            //update telemetry
            telemetry.addData("targetVelocity", motionState.v)
            telemetry.addData("measuredVelocity", currentVelo)
            telemetry.addData("error", motionState.v - currentVelo)

            telemetry.update()
        }
    }
}