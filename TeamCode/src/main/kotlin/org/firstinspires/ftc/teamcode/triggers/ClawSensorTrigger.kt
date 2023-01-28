package org.firstinspires.ftc.teamcode.triggers

import com.arcrobotics.ftclib.command.button.Trigger
import com.qualcomm.hardware.rev.RevColorSensorV3
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTelemetry
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class ClawSensorTrigger(private val colorSensor: RevColorSensorV3, private val telemetry: Telemetry) : Trigger() {
    override fun get(): Boolean {
        return colorSensor.red() > 1000 && colorSensor.getDistance(DistanceUnit.CM) < 1.0
    }
}