package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

@Autonomous(name = "blue auto")
@Disabled
public class BlueAuto extends LinearOpMode {
    DcMotor backleftDrive;
    DcMotor backrightDrive;
    DcMotor frontleftDrive;
    DcMotor frontrightDrive;
    @Override
    public void runOpMode() throws InterruptedException {
        backleftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backrightDrive = hardwareMap.get(DcMotor.class, "backRight");
        frontleftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontrightDrive = hardwareMap.get(DcMotor.class, "frontRight");

        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);

        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        backrightDrive.setPower(0.3);
        backleftDrive.setPower(0.3);
        frontrightDrive.setPower(0.3);
        frontleftDrive.setPower(0.3);

        sleep(2750);

        backrightDrive.setPower(-0.3);
        backleftDrive.setPower(0.3);
        frontrightDrive.setPower(0.3);
        frontleftDrive.setPower(-0.3);

        sleep(2000);
    }
}
