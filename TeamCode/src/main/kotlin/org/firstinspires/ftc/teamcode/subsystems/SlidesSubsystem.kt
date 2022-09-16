package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import org.firstinspires.ftc.teamcode.constants.SlidesConst.SlidesFeedforward.*
import org.firstinspires.ftc.teamcode.constants.SlidesConst.SlidesPID.*
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kA
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kV

class SlidesSubsystem(slidesLeft: Motor, slidesRight: Motor): SubsystemBase() {
    private val slideMotors = MotorGroup(slidesLeft, slidesRight)

    companion object {
        private val slidesFeedforward = ElevatorFeedforward(
            Ks.coeff,
            Kg.coeff,
            Kv.coeff,
            Ka.coeff
        )
        private val slidesPID = PIDFController(
            P.coeff,
            I.coeff,
            D.coeff,
            0.0,
        )
    }

    fun slideUp() = slideMotors.set(0.2)

    fun slideDown() = slideMotors.set(-0.2)

    fun stop() = slideMotors.stopMotor()
}