package org.firstinspires.ftc.teamcode.opModes.config.slides

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.util.NanoClock
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.constants.DeviceConfig
import org.firstinspires.ftc.teamcode.constants.SlidesConst.SlidesConstraints.MAX_ACCELERATION
import org.firstinspires.ftc.teamcode.constants.SlidesConst.SlidesConstraints.MAX_VELOCITY
import org.firstinspires.ftc.teamcode.subsystems.SlidesClawSubsystem

@Disabled
@Autonomous(group = "Slides Tuning")
@Config
class SlidesFeedforwardTuner(): LinearOpMode() {
    companion object {
        @JvmField
        var DISTANCE : Double = 3200.0 //encoder ticks

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
        var kS: Double = 0.0

        @JvmField
        var kV: Double = 0.000467

        @JvmField
        var kA: Double = 0.0
    }

    private lateinit var slidesLeft: Motor
    private lateinit var slidesRight: Motor
    private lateinit var limit: TouchSensor
    private lateinit var clawServo: Servo
    private lateinit var rotationServo: Servo

    private lateinit var subsystem: SlidesClawSubsystem
    private val dashboard = FtcDashboard.getInstance()

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        slidesLeft = Motor(hardwareMap, DeviceConfig.SLIDES_LEFT.deviceName, Motor.GoBILDA.RPM_1150)
        slidesRight = Motor(hardwareMap, DeviceConfig.SLIDES_RIGHT.deviceName, Motor.GoBILDA.RPM_1150)
        limit = hardwareMap.get(TouchSensor::class.java, DeviceConfig.SLIDES_LIMIT.deviceName)
        clawServo = hardwareMap.get(Servo::class.java, DeviceConfig.CLAW_SERVO.deviceName)
        rotationServo = hardwareMap.get(Servo::class.java, DeviceConfig.ROTATION_SERVO.deviceName)


        subsystem = SlidesClawSubsystem(slidesLeft, slidesRight, clawServo, rotationServo, limit, telemetry)

        val clock = NanoClock.system()

        telemetry.addLine("Ready!")
        telemetry.update()
        telemetry.clearAll()

        waitForStart()

        if(isStopRequested) return

        var forward = true
        var activeProfile = generateProfile(true)
        var profileStart = clock.seconds()

        while(!isStopRequested) {
            val profileTime = clock.seconds() - profileStart

            if(profileTime > activeProfile.duration()) {
                forward = !forward
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