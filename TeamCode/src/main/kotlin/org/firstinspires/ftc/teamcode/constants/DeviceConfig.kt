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
    ODO_STRAFE_ENCODER("intake"), //C-HUB 1
    ODO_RIGHT_ENCODER("rightRear"), //C-HUB 3
    ODO_LEFT_ENCODER("rightFront"), //C-HUB 0

    //Elevator Motors
    SLIDES_LEFT("leftSlide"),
    SLIDES_RIGHT("rightSlide"),
    SLIDES_LIMIT("slidesLimit")
}