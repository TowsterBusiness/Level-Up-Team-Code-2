package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;

public static class Robot {
    public static HardwareMap hardwareMap = null;
    public static DcMotor backRightDrive = null;
    public static DcMotor backLefttDrive = null;
    public static DcMotor frontRightDrive = null;
    public static DcMotor frontLeftDrive = null;
    public static int backRightCoefficient = 1;
    public static int backLeftCoefficient = 1;
    public static int frontRightCoefficient = 1;
    public static int frontLeftCoefficient = 1;
    

    public static forward(int amount) {
        if (hardwareMap == null || frontRightDrive == null || frontLeftDrive == null || backRightDrive == null || backLeftDrive == null) return;

        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + amount * backRightCoefficient);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + amount * backLeftCoefficient);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + amount * frontRightCoefficient);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + amount * frontLeftCoefficient);
    }

    public static backward(int amount) {
        if (hardwareMap == null || frontRightDrive == null || frontLeftDrive == null || backRightDrive == null || backLeftDrive == null) return;

        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() - amount * backRightCoefficient);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() - amount * backLeftCoefficient);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() - amount * frontRightCoefficient);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - amount * frontLeftCoefficient);
    }

    //NOTE: DO THESE THINGS BELOW
    public static right(int amount) {
        if (hardwareMap == null || frontRightDrive == null || frontLeftDrive == null || backRightDrive == null || backLeftDrive == null) return;

        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() - amount * backRightCoefficient);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() - amount * backLeftCoefficient);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() - amount * frontRightCoefficient);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - amount * frontLeftCoefficient);
    }

    public static left(int amount) {
        if (hardwareMap == null || frontRightDrive == null || frontLeftDrive == null || backRightDrive == null || backLeftDrive == null) return;

        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() - amount * backRightCoefficient);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() - amount * backLeftCoefficient);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() - amount * frontRightCoefficient);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - amount * frontLeftCoefficient);
    }

    public static turnRight(int amount) {
        if (hardwareMap == null || frontRightDrive == null || frontLeftDrive == null || backRightDrive == null || backLeftDrive == null) return;

        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() - amount * backRightCoefficient);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() - amount * backLeftCoefficient);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() - amount * frontRightCoefficient);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - amount * frontLeftCoefficient);
    }

    public static turnLeft(int amount) {
        if (hardwareMap == null || frontRightDrive == null || frontLeftDrive == null || backRightDrive == null || backLeftDrive == null) return;

        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() - amount * backRightCoefficient);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() - amount * backLeftCoefficient);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() - amount * frontRightCoefficient);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - amount * frontLeftCoefficient);
    }
}