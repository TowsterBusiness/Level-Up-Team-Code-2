package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

@Autonomous(name = "wheelstest")

public class TestWheels extends LinearOpMode {
    DcMotor backleftDrive;
    DcMotor backrightDrive;
    DcMotor frontleftDrive;
    DcMotor frontrightDrive;

    Servo claw;
    float clawPosition = 0;
    ButtonToggle clawToggle = new ButtonToggle();
    Servo clawHinge;
    float clawHingePosition = 0;
    public IMU imu;

    boolean isClawOpen = false;

    public double angle;

    public DcMotor arm1;
    public DcMotor arm2;

    public double armAngle1 = 2357;
    public double armAngle2 = 650;

    double powerVar = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        backleftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backrightDrive = hardwareMap.get(DcMotor.class, "backRight");
        frontleftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontrightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        arm1 = hardwareMap.get(DcMotor.class, "baseArm");
        arm2 = hardwareMap.get(DcMotor.class, "floatingArm");
        claw = hardwareMap.get(Servo.class, "claw");
        clawHinge = hardwareMap.get(Servo.class, "clawHinge");

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

        arm1.setTargetPosition((int) armAngle1);
        arm2.setTargetPosition((int) armAngle2);

        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );

        //This is the potential imu starting program

        telemetry.addData("Direction Mode:", "Back Mode");
        imu.initialize(parameters);

        waitForStart();


        while (opModeIsActive()) {

            YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw", robotOrientation.getYaw(AngleUnit.DEGREES));

            backrightDrive.setPower(gamepad1.dpad_left ? 0.35 : 0);
            backleftDrive.setPower(gamepad1.dpad_down ? 0.35 : 0);
            frontrightDrive.setPower(gamepad1.dpad_up ? 0.35 : 0);
            frontleftDrive.setPower(gamepad1.dpad_right ? 0.35 : 0);

            // clawHinge Movement
            clawHingePosition += gamepad2.right_stick_y * 0.005;
            clawHinge.setPosition(clawHingePosition);
            telemetry.addData("claw hinge position", clawHingePosition);


            // Claw Movement
            clawToggle.update(gamepad2.a);
            float clawOpenPosition = 0.1f;
            float clawClosedPosition = 0.38f;
            if (gamepad2.a) {
                claw.setPosition(clawOpenPosition);
            } else {
                claw.setPosition(clawClosedPosition);
            }

            double y = gamepad1.left_stick_y;
            double x = gamepad1.right_stick_y;
            armAngle1 += y * 0.5f;
            armAngle2 += x * 0.5f;
            arm1.setTargetPosition((int) (armAngle1 * 14.45277777f));
            arm2.setTargetPosition((int) (armAngle2 * -3.33333333333 + 612));
            telemetry.addData("Arm Position (angle): ", armAngle1);
            telemetry.addData("Arm2 Position (angle): ", armAngle2);
            telemetry.addData("Arm Encoder: ", arm1.getCurrentPosition());
            telemetry.addData("Arm2 Position: ", arm2.getCurrentPosition());

            telemetry.update();

        }
    }
}