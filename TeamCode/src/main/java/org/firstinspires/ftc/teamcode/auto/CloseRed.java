package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.RobotMovement;

@Autonomous(name = "Close Red")
public class CloseRed extends LinearOpMode {
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
        backleftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backrightDrive = hardwareMap.get(DcMotor.class, "backRight");
        frontleftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontrightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        arm1 = hardwareMap.get(DcMotor.class, "baseArm");
        arm2 = hardwareMap.get(DcMotor.class, "floatingArm");

        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        backleftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontleftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftDrive.setTargetPosition(0);
        frontrightDrive.setTargetPosition(0);
        backleftDrive.setTargetPosition(0);
        backrightDrive.setTargetPosition(0);

        frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RobotMovement.hardwareMap = hardwareMap;
        RobotMovement.backLeftDrive = backleftDrive;
        RobotMovement.backRightDrive = backrightDrive;
        RobotMovement.frontLeftDrive = frontleftDrive;
        RobotMovement.frontRightDrive = frontrightDrive;

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "claw");
        clawHinge = hardwareMap.get(Servo.class, "clawHinge");

        claw.setPosition(0.38);

        waitForStart();

        backrightDrive.setPower(3.5);
        backleftDrive.setPower(3.5);
        frontrightDrive.setPower(3.5);
        frontleftDrive.setPower(3.5);

        int amount = 1280;
        RobotMovement.forward(1280);

        sleep(1000);

        int line = 1;
        if (line == 1 || line == 3) {
            RobotMovement.forward(200);
            sleep(500);
            if (line == 1) {
                RobotMovement.turnLeft(900);
                sleep(1000);
            } else if (line == 3) {
                RobotMovement.turnRight(900);
                sleep(1000);
            }
        }

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

        arm1.setPower(0.3);
        arm2.setPower(0.1);
        arm1.setTargetPosition(2357);
        arm2.setTargetPosition(640);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(4000);
        clawHinge.setPosition(0.65);

        sleep(1000);

        claw.setPosition(0.1f);

        sleep(1000);

        if (line == 1 || line == 3) {
            if (line == 1) {
                RobotMovement.turnRight(900);
                sleep(1000);
            } else if (line == 3) {
                RobotMovement.turnLeft(900);
                sleep(1000);
            }
            RobotMovement.backward(200);
            sleep(500);
        }


        RobotMovement.turnRight(900);
        sleep(1000);

        RobotMovement.forward(2560);
        sleep(2000);



//        backrightDrive.setPower(0);
//        backleftDrive.setPower(0);
//        frontrightDrive.setPower(0);
//        frontleftDrive.setPower(0);
//
//        if (line == 1 || line == 3) {
//            if (line == 3) {
//                backrightDrive.setPower(-0.3);
//                backleftDrive.setPower(0.3);
//                frontrightDrive.setPower(-0.3);
//                frontleftDrive.setPower(0.3);
//
//                sleep(1500);
//
//
//
//                backrightDrive.setPower(0.35);
//                backleftDrive.setPower(0.35);
//                frontrightDrive.setPower(0.35);
//                frontleftDrive.setPower(0.35);
//
//                sleep(245);
//            } else {
//                backrightDrive.setPower(0.3);
//                backleftDrive.setPower(-0.3);
//                frontrightDrive.setPower(0.3);
//                frontleftDrive.setPower(-0.3);
//
//                sleep(1500);
//
//                backrightDrive.setPower(-0.35);
//                backleftDrive.setPower(-0.35);
//                frontrightDrive.setPower(-0.35);
//                frontleftDrive.setPower(-0.35);
//                sleep(285);
//
//
//            }
//
//
//        }
//        backrightDrive.setPower(0);
//        backleftDrive.setPower(0);
//        frontrightDrive.setPower(0);
//        frontleftDrive.setPower(0);
//
//
//        arm1.setPower(0.3);
//        arm2.setPower(0.1);
//        arm1.setTargetPosition(1504);
//        arm2.setTargetPosition(840);
//        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        sleep(4000);
//        clawHinge.setPosition(0.65);
//
//        sleep(1000);
//
//        claw.setPosition(0.1f);
//
//        sleep(1000);
//
//        arm1.setPower(0.3);
//        arm2.setPower(0.1);
//        arm1.setTargetPosition(2357);
//        arm2.setTargetPosition(640);
//        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        sleep(4000);
//        clawHinge.setPosition(0.65);
//
//        sleep(1000);
//
//        claw.setPosition(0.1f);
//
//        sleep(1000);
//
//        if (line == 1 || line == 3) {
//            if (line == 3) {
//                backrightDrive.setPower(0.3);
//                backleftDrive.setPower(-0.3);
//                frontrightDrive.setPower(0.3);
//                frontleftDrive.setPower(-0.3);
//            } else {
//                backrightDrive.setPower(-0.3);
//                backleftDrive.setPower(0.3);
//                frontrightDrive.setPower(-0.3);
//                frontleftDrive.setPower(0.3);
//            }
//            sleep(1500);
//        }
//
//        backrightDrive.setPower(-0.35);
//        backleftDrive.setPower(-0.35);
//        frontrightDrive.setPower(-0.35);
//        frontleftDrive.setPower(-0.35);
//
//        sleep(2000);
//
//        backrightDrive.setPower(0.3);
//        backleftDrive.setPower(-0.3);
//        frontrightDrive.setPower(-0.3);
//        frontleftDrive.setPower(0.3);
//
//        sleep(4150);
//
//


    }
}