package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name = "wheelstest")

public class TestWheels extends LinearOpMode {
    private DcMotor backleftDrive = null;
    private DcMotor newMotor = null;
    private DcMotor newNewMotor = null;

    double powerVar = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        backleftDrive = hardwareMap.get(DcMotor.class, "m1");
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backleftDrive.setTargetPosition(0);

        backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftDrive.setPower(powerVar);
        newMotor = hardwareMap.get(DcMotor.class, "m5");
        newMotor.setDirection(DcMotor.Direction.REVERSE);
        newMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        newMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newMotor.setTargetPosition(0);

        newMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        newMotor.setPower(powerVar);
        newNewMotor = hardwareMap.get(DcMotor.class, "m6");
        newNewMotor.setDirection(DcMotor.Direction.REVERSE);
        newNewMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        newNewMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newNewMotor.setTargetPosition(0);

        newNewMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        newNewMotor.setPower(powerVar);

        waitForStart();

        backleftDrive.setTargetPosition(10000);

    }

}