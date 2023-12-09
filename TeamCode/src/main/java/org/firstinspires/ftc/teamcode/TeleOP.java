package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

@TeleOp(name = "teleop")
public class TeleOP extends LinearOpMode {
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


    @Override
    public void runOpMode() throws InterruptedException {
        // This is the start of the program

        // We have is setup that the motors that are connected have name to them. This asks the robot for those motors that we named so we can control them
        backleftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backrightDrive = hardwareMap.get(DcMotor.class, "backRight");
        frontleftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontrightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        // the bottom section of the arm. Will be refered at arm1 sometimes
        arm1 = hardwareMap.get(DcMotor.class, "baseArm");
        // the top section of the arm. Will be refered at arm2 sometimes
        arm2 = hardwareMap.get(DcMotor.class, "floatingArm");
        claw = hardwareMap.get(Servo.class, "claw");
        clawHinge = hardwareMap.get(Servo.class, "clawHinge");

        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);

        // This sets what happends when the robot motors aren't moving. for this case, we make it break so it doesn't move from it's spot
        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // There are 2 ways the motors can be controled. by setting the speed of the motor, or by setting the position that the motor needs to go. This is the latter
        arm1.setTargetPosition((int) armAngle1);
        arm2.setTargetPosition((int) armAngle2);

        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // The imu gets the velocity and rotation of the robot
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );

        // telemetry is like a print statement but for robotics. It shows the data on the screen

        telemetry.addData("Direction Mode:" , "Back Mode");
        imu.initialize(parameters);

        // There are 2 states a program to be in. When the button if first pressed, it runs the code above until we press it the second time, running the code below
        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.y) {
                imu.initialize(parameters);
            }
            

            YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw", robotOrientation.getYaw(AngleUnit.DEGREES));

            angle = robotOrientation.getYaw(AngleUnit.RADIANS);

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            // this is what we call voodoo magic math coding. I kinda understand it and if you want to understand it, you can look up "mechanum wheel drive"
            // rotates the left stick and changes the x & y values accordingly (field centric)
            Vector2D inputVector = new Vector2D(x, y);
            Vector2D rotatedVector = inputVector.rotateVector(-angle);

            x = rotatedVector.x;
            y = rotatedVector.y;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.1);
            double frontleftPower = (y + x + rx) / denominator;
            double backleftPower = (y - x + rx) / denominator;
            double frontrightPower = (y - x - rx) / denominator;
            double backrightPower = (y + x - rx) / denominator;

            // pressurising the right trigger slows down the drive train
            double coefficient = 0.35;
            double coefficientArm = 1;
            if(gamepad1.right_trigger < 0.5)
            {
                telemetry.addData("Drive Speed Mode", "off");
            }
            else
            {
                telemetry.addData("Drive Speed Mode", "trigger on");
                coefficient = 1;
            }

            if(gamepad2.right_trigger < 0.5)
            {
                telemetry.addData("Arm Speed Mode", "off");
            }
            else
            {
                telemetry.addData("Arm Speed Mode", "trigger on");
                coefficientArm = 1.5f;
            }
            arm1.setPower(0.3 * coefficientArm);
            arm2.setPower(0.1 * coefficientArm);

            telemetry.addData("Front Left Power", frontleftPower * coefficient);

            backrightDrive.setPower(backrightPower * coefficient);
            backleftDrive.setPower(backleftPower * coefficient);
            frontrightDrive.setPower(frontrightPower * coefficient);
            frontleftDrive.setPower(frontleftPower * coefficient);

            // clawHinge Movement
            clawHingePosition += gamepad2.right_stick_y * 0.005;

           float clawHingeUpperBound = 1;
           float clawHingeLowerBound = 0.66f;
           if (clawHingePosition >= clawHingeUpperBound) {
               clawHingePosition = clawHingeUpperBound;
           } else if (clawPosition <= clawHingeLowerBound) {
               clawPosition = clawHingeLowerBound;
           }
           telemetry.addData("y3", clawHingePosition);
           clawHinge.setPosition(clawHingePosition);

            // Claw Movement
            clawToggle.update(gamepad2.a);
            float clawOpenPosition = 0.1f;
            float clawClosedPosition = 0.38f;
            if (gamepad2.a) {
                claw.setPosition(clawOpenPosition);
            } else {
                claw.setPosition(clawClosedPosition);
            }


            if (gamepad2.x) {
                // pick up position
                armAngle1 = 1504;
                armAngle2 = 840;
                clawHingePosition = 0.65f;
            } else if (gamepad2.y) {
                // compact position
                armAngle1 = 2357;
                armAngle2 = 650;
                clawHingePosition = 0.65f;
            } else if (gamepad2.b) {
                // drop position
                armAngle1 = 1616;
                armAngle2 = 136;
                clawHingePosition = 0.22f;
            } else if (gamepad2.dpad_up) {
                // hang start position
                armAngle1 = 1280;
                armAngle2 = 333;
            } else if (gamepad2.dpad_down) {
                // hanging position
                armAngle1 = 529;
                armAngle2 = 130;
            } else if (gamepad1.back) {
                // hard restart position
                armAngle1 = 0;
                armAngle2 = 0;
            }

            arm1.setTargetPosition((int) armAngle1);
            arm2.setTargetPosition((int) armAngle2);
            clawHinge.setPosition(clawHingePosition);

            telemetry.update();

        }

    }

}