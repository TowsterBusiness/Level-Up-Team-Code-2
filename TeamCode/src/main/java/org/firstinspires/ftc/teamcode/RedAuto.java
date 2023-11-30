package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

@Autonomous(name = "red auto")
public class RedAuto extends LinearOpMode {
    DcMotor backleftDrive;
    DcMotor backrightDrive;
    DcMotor frontleftDrive;
    DcMotor frontrightDrive;

    Servo claw;
    Servo clawHinge;

    DcMotor arm1;
    DcMotor arm2;
    @Override
    public void runOpMode() throws InterruptedException {
        backleftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backrightDrive = hardwareMap.get(DcMotor.class, "backRight");
        frontleftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontrightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        arm1 = hardwareMap.get(DcMotor.class, "baseArm");
        arm2 = hardwareMap.get(DcMotor.class, "floatingArm");

        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);

        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "claw");
        clawHinge = hardwareMap.get(Servo.class, "clawHinge");

        claw.setPosition(0.38);



        waitForStart();

        backrightDrive.setPower(0.35);
        backleftDrive.setPower(0.35);
        frontrightDrive.setPower(0.35);
        frontleftDrive.setPower(0.35);

        sleep(1800);

        backrightDrive.setPower(0);
        backleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        frontleftDrive.setPower(0);

        arm1.setPower(0.3);
        arm2.setPower(0.1);
        arm1.setTargetPosition(1504);
        arm2.setTargetPosition(840);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(4000);
        clawHinge.setPosition(0.65);

        sleep(1000);

        claw.setPosition(0.1f);

        sleep(1000);

        backrightDrive.setPower(-0.35);
        backleftDrive.setPower(-0.35);
        frontrightDrive.setPower(-0.35);
        frontleftDrive.setPower(-0.35);

        sleep(500);

        backrightDrive.setPower(0.3);
        backleftDrive.setPower(-0.3);
        frontrightDrive.setPower(-0.3);
        frontleftDrive.setPower(0.3);

        sleep(2000);
    }
}
