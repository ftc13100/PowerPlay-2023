package org.firstinspires.ftc.teamcode.triggers

import com.arcrobotics.ftclib.command.button.Trigger
import com.qualcomm.hardware.rev.RevColorSensorV3
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class ClawSensorTrigger(private val colorSensor: RevColorSensorV3) : Trigger() {
    override fun get(): Boolean {
        return colorSensor.red() > 0.5 && colorSensor.getDistance(DistanceUnit.CM) < 5.0
    }
}