package org.firstinspires.ftc.teamcode.opModes.teleOp.tests.slides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.constants.DeviceConfig;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;

@TeleOp
public class SlidesPIDTuner extends CommandOpMode {
    //hardware
    private Motor leftMotor;
    private Motor rightMotor;
    private TouchSensor limit;

    //subsystems
    private SlidesSubsystem slidesSubsystem;

    //gamepads

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);

        this.leftMotor = new Motor (hardwareMap, DeviceConfig.SLIDES_LEFT.getDeviceName());
        this.rightMotor = new Motor(hardwareMap, DeviceConfig.SLIDES_RIGHT.getDeviceName());
        this.limit = hardwareMap.get(TouchSensor.class, DeviceConfig.SLIDES_LIMIT.getDeviceName());

        this.slidesSubsystem = new SlidesSubsystem(leftMotor, rightMotor, limit, telemetry);

        register(slidesSubsystem);
        slidesSubsystem.setDefaultCommand(new RunCommand(slidesSubsystem::operateSlides, slidesSubsystem));

    }
    // makme a perptual comann taht run slides on slides subsystem
}
