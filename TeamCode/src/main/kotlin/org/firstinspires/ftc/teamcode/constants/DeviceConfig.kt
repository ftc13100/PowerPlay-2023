package org.firstinspires.ftc.teamcode.constants

enum class DeviceConfig(val deviceName: String) {
    //Vision
    VISION_CAMERA("lifecam"),

    //intake
    INTAKE("intake"),

    //Drive Motors
    DRIVE_LEFT_REAR("leftRear"),
    DRIVE_RIGHT_REAR("rightRear"),
    DRIVE_RIGHT_FRONT("rightFront"),
    DRIVE_LEFT_FRONT("leftFront"),

    //Odo encoders
    ODO_STRAFE_ENCODER("strafeEncoder"),
    ODO_RIGHT_ENCODER("rightRear"),
    ODO_LEFT_ENCODER("rightFront"),

    //Elevator Motors
    SLIDES_RIGHT("slidesRight"),
    SLIDES_LEFT("slidesLeft")
}