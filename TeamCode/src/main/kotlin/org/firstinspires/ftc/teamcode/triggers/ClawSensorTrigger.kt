package org.firstinspires.ftc.teamcode.triggers

import com.arcrobotics.ftclib.command.button.Trigger
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class ClawSensorTrigger(private val color: ColorSensor, private val distance: DistanceSensor) : Trigger() {
    override fun get(): Boolean {
        return color.red() > 0.5 && distance.getDistance(DistanceUnit.CM) < 5.0
    }
}