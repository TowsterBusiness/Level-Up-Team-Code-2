package org.firstinspires.ftc.teamcode.configuration;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@TeleOp(name = "Encoder Config")
public class EncoderConfig extends LinearOpMode {
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

    public double armAngle1 = 0;
    public double armAngle2 = 0;

    PIDController pid1 = new PIDController(0.8f, 0f, 0f);
    PIDController pid2 = new PIDController(0.8f, 0f, 0f);
    ElapsedTime et = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        backleftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        backrightDrive = hardwareMap.get(DcMotor.class, "rightRear");
        frontleftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontrightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        arm1 = hardwareMap.get(DcMotor.class, "baseArm");
        arm2 = hardwareMap.get(DcMotor.class, "floatingArm");
        claw = hardwareMap.get(Servo.class, "claw");
        clawHinge = hardwareMap.get(Servo.class, "clawHinge");

        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );

        //This is the potential imu starting program

        telemetry.addData("Direction Mode:" , "Back Mode");
        imu.resetYaw();
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y) {
                imu.resetYaw();
                imu.initialize(parameters);
            }

            YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw", robotOrientation.getYaw(AngleUnit.DEGREES));

            angle = robotOrientation.getYaw(AngleUnit.RADIANS);

            double frontleftPower = gamepad1.dpad_left ? 1 : 0;
            double backleftPower = gamepad1.dpad_down ? 1 : 0;
            double frontrightPower = gamepad1.dpad_up ? 1 : 0;
            double backrightPower = gamepad1.dpad_right ? 1 : 0;

            telemetry.addData("fl (d_pad left)", gamepad1.dpad_left);
            telemetry.addData("bl (d_pad down)", gamepad1.dpad_down);
            telemetry.addData("fr (d_pad up)", gamepad1.dpad_up);
            telemetry.addData("br (d_pad right)", gamepad1.dpad_right);

            telemetry.addData("fl Encoder", frontleftDrive.getCurrentPosition());
            telemetry.addData("bl Encoder", backleftDrive.getCurrentPosition());
            telemetry.addData("fr Encoder", frontrightDrive.getCurrentPosition());
            telemetry.addData("br Encoder", backrightDrive.getCurrentPosition());

            backrightDrive.setPower(backrightPower);
            backleftDrive.setPower(backleftPower);
            frontrightDrive.setPower(frontrightPower);
            frontleftDrive.setPower(frontleftPower);


            telemetry.update();

        }

    }

}