package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.RobotMovement;

@Autonomous(name = "Close Blue")
public class CloseBlue extends LinearOpMode {
    DcMotor backleftDrive;
    DcMotor backrightDrive;
    DcMotor frontleftDrive;
    DcMotor frontrightDrive;

    Servo claw;
    Servo clawHinge;

    DcMotor arm1;
    DcMotor arm2;

    DistanceSensor ds;
    @Override
    public void runOpMode() throws InterruptedException {
        backleftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        backrightDrive = hardwareMap.get(DcMotor.class, "rightRear");
        frontleftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        frontrightDrive = hardwareMap.get(DcMotor.class, "leftFront");
        ds = hardwareMap.get(DistanceSensor.class, "ds");
        arm1 = hardwareMap.get(DcMotor.class, "baseArm");
        arm2 = hardwareMap.get(DcMotor.class, "floatingArm");

        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        backrightDrive.setPower(0.5);
        backleftDrive.setPower(0.5);
        frontrightDrive.setPower(0.5);
        frontleftDrive.setPower(0.5);

        RobotMovement.left(510);
        sleep(1500);
        RobotMovement.forward(1180);
        clawHinge.setPosition(0.65f);
        sleep(1000);
        RobotMovement.forward(1500);

        double min = 999999;
        while (backleftDrive.isBusy() || frontrightDrive.isBusy()) {
            double distance = ds.getDistance(DistanceUnit.CM);
            min = Math.min(distance, min);
            telemetry.addData("Distance", distance);
            telemetry.addData("Min", min);
            telemetry.update();
        }

        sleep(10);
        RobotMovement.backward(700);
        sleep(2000);
        RobotMovement.turnLeft(900);
        sleep(1500);
        RobotMovement.forward(300);

        double min2 = 999999;
        while (backleftDrive.isBusy() || frontrightDrive.isBusy()) {
            double distance = ds.getDistance(DistanceUnit.CM);
            min2 = Math.min(distance, min2);
            telemetry.addData("Distance", distance);
            telemetry.addData("Min", min);
            telemetry.addData("Min2", min2);
            telemetry.update();
        }

        sleep(10);
        RobotMovement.forward(200);
        sleep(1000);
        RobotMovement.backward(500);
        sleep(1000);
        RobotMovement.turnRight(900);
        clawHinge.setPosition(0);
        sleep(1500);
        RobotMovement.backward(1890);
        sleep(2500);
        RobotMovement.left(510);
        sleep(1000);

        int line = 1;
        if (min <= 10) {
            line = 3;
        } else if (min2 <= 18) {
            line = 2;
        }

        int amount = 1280;
        RobotMovement.forward(1380);

        sleep(1200);


        if (line == 1 || line == 3) {
            RobotMovement.forward(200);
            sleep(500);
            if (line == 1) {
                RobotMovement.turnLeft(900);
                sleep(1000);
                RobotMovement.backward(20);
                sleep(500);
            } else if (line == 3) {
                RobotMovement.turnRight(880);
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
        clawHinge.setPosition(0.65);

        sleep(3000);

        claw.setPosition(0.1f);

        sleep(1000);

        if (line == 1) {
            RobotMovement.turnRight(900);
            sleep(1000);
        } else if (line == 3) {
            RobotMovement.turnLeft(900);
            sleep(1000);
        } else {
            RobotMovement.backward(200);
            sleep(500);
        }

        RobotMovement.right(2560);
        sleep(2000);
    }
}