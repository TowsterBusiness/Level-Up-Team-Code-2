package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

@TeleOp(name = "teleop")
public class TeleOP extends OpMode {
    DcMotor backleftDrive;
    DcMotor backrightDrive;
    DcMotor frontleftDrive;
    DcMotor frontrightDrive;

    Servo claw;
    float clawPosition = 0;
    ButtonToggle clawToggle = new ButtonToggle();
    Servo clawHinge;
    Servo droneHinge;
    float clawHingePosition = 0;
    public IMU imu;

    boolean isClawOpen = false;

    public double angle;

    public DcMotor arm1;
    public DcMotor arm2;

    public double armAngle1 = 0;
    public double armAngle2 = 0;

    PIDController pid1 = new PIDController(0.8f, 0f, 0f);
    PIDController pid2 = new PIDController(0.8f, 0f, 0f);
    ElapsedTime et = new ElapsedTime();

    IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
    );


    @Override
    public void init() {
        backleftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        backrightDrive = hardwareMap.get(DcMotor.class, "rightRear");
        frontleftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontrightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        arm1 = hardwareMap.get(DcMotor.class, "baseArm");
        arm2 = hardwareMap.get(DcMotor.class, "floatingArm");
        claw = hardwareMap.get(Servo.class, "claw");
        clawHinge = hardwareMap.get(Servo.class, "clawHinge");
        droneHinge = hardwareMap.get(Servo.class, "droneHinge");
        droneHinge.setPosition(0.7);

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

        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");



        //This is the potential imu starting program

        telemetry.addData("Direction Mode:" , "Back Mode");
        imu.resetYaw();
        imu.initialize(parameters);
    }

    @Override
    public void start() {
        super.start();
        et.reset();
    }

    @Override
    public void loop() {
        float dt = (float) et.milliseconds();
        et.reset();

        if (gamepad1.y) {
            imu.resetYaw();
            imu.initialize(parameters);
        }
        if (gamepad1.back) {
            armAngle1 = 0;
            armAngle2 = 0;
        }

        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw", robotOrientation.getYaw(AngleUnit.DEGREES));

        angle = robotOrientation.getYaw(AngleUnit.RADIANS);

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

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
            telemetry.addData("Yeet Mode", "off");
        }
        else
        {
            telemetry.addData("YeetMode", "trigger on");
            coefficient = 1;
        }
//
//            {
//                telemetry.addData("Yeet Mode2", "off");//            if(gamepad2.right_trigger < 0.5)
//            }
//            else
//            {
//                telemetry.addData("YeetMode2", "trigger on");
//                coefficientArm = 1.5f;
//            }
//            arm1.setPower(0.8 * coefficientArm);
//            arm2.setPower(0.8 * coefficientArm);

        telemetry.addData("Front Left Power", frontleftPower * coefficient);

        backrightDrive.setPower(backrightPower * coefficient);
        backleftDrive.setPower(backleftPower * coefficient);
        frontrightDrive.setPower(frontrightPower * coefficient);
        frontleftDrive.setPower(frontleftPower * coefficient);

        // clawHinge Movement
        clawHingePosition += gamepad2.right_stick_y * 0.005;

//            float clawHingeUpperBound = 1;
//            float clawHingeLowerBound = 0.66f;
//            if (clawHingePosition >= clawHingeUpperBound) {
//                clawHingePosition = clawHingeUpperBound;
//            } else if (clawPosition <= clawHingeLowerBound) {
//                clawPosition = clawHingeLowerBound;
//            }
//            telemetry.addData("y3", clawHingePosition);
//            clawHinge.setPosition(clawHingePosition);

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
            armAngle1 = 1194;//1348;//1537;
            armAngle2 = 829;//843;//860;
            clawHingePosition = 0.422f;//0.592f;
        } else if (gamepad2.y) {
            armAngle1 = 2262;//2357;
            armAngle2 = 627;//650;
            clawHingePosition = 0.66f;
        } else if (gamepad2.b) {
            armAngle1 = 1795;//1616;//1993;
            armAngle2 = 99;//136;//241;
            clawHingePosition = 0f;
        } else if (gamepad2.dpad_up) {
            //1280 333 1976 440
            armAngle1 = 1280;
            armAngle2 = 333;
        } else if (gamepad2.dpad_down) {
            armAngle1 = 529;//1976;
            armAngle2 = 130;//440;
        }
        if (gamepad2.left_trigger > 0.5) {
            droneHinge.setPosition(0.2);
        }

        telemetry.addData("drone pos: ", droneHinge.getPosition());

        float v1 = pid1.update((float) (arm1.getCurrentPosition() - armAngle1) / 14.45277777f, dt);
        float v2 = pid2.update((float) (arm2.getCurrentPosition() - armAngle2) / -3.33333333f, dt);

        arm1.setPower(v1 / -17 / 4);
        arm2.setPower(v2 / 204 / 4);

        telemetry.addData("v1: ", v1 / -17 / 4);
        telemetry.addData("v2: ", v2 / 250 / 4);

        clawHinge.setPosition(clawHingePosition);
        telemetry.update();

    }
}