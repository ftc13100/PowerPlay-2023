package org.firstinspires.ftc.teamcode.constants

enum class DeviceConfig(val deviceName: String) {
    // Sensors
    VISION_CAMERA("lifecam"),
    COLOR_SENSOR("colorSensor"),

    // Intake
    INTAKE("intake"),

    // Claw Intake
    CLAW_SERVO("clawServo"),
    ROTATION_SERVO("rotationServo"),

    // Drivetrain Motors
    DRIVE_LEFT_REAR("leftRear"),
    DRIVE_RIGHT_REAR("rightRear"),
    DRIVE_RIGHT_FRONT("rightFront"),
    DRIVE_LEFT_FRONT("leftFront"),

    // Encoders
    ODO_STRAFE_ENCODER("rightRear"), // C-HUB 1
    ODO_RIGHT_ENCODER("leftFront"), // C-HUB 3
    ODO_LEFT_ENCODER("rightFront"), // C-HUB 0

    // Lift Motors
    SLIDES_LEFT("leftSlide"),
    SLIDES_RIGHT("rightSlide"),
    SLIDES_LIMIT("slidesLimit")
}