package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit


class IntakeSubsystem(private val intake: MotorEx) : SubsystemBase() {

    val current
        get() = intake.motorEx.getCurrent(CurrentUnit.AMPS)

    var currentAlert
        get() = intake.motorEx.getCurrentAlert(CurrentUnit.AMPS)
        set(value) = intake.motorEx.setCurrentAlert(value, CurrentUnit.AMPS)

    fun intake() = intake.set(-1.0)

    fun outtake() = intake.set(1.0)

    fun stop() = intake.stopMotor()
}