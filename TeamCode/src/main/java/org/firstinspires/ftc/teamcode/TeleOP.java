package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

@TeleOp(name="teleop")
public class TeleOP extends LinearOpMode {
    DcMotor backleftDrive;
    DcMotor backrightDrive;
    DcMotor frontleftDrive;
    DcMotor frontrightDrive;

    public BNO055IMU imu;

    public double angle;

    @Override
    public void runOpMode() throws InterruptedException {
        backleftDrive = hardwareMap.get(DcMotor.class, "m1");
        backrightDrive = hardwareMap.get(DcMotor.class, "m2");
        frontleftDrive = hardwareMap.get(DcMotor.class, "m3");
        frontrightDrive = hardwareMap.get(DcMotor.class, "m4");

        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);

        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        //This is the potential imu starting program

        telemetry.addData("Direction Mode:" , "Back Mode");
        imu.initialize(parameters);

        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("direction", imu.getAngularOrientation());

            if(gamepad1.y)
            {
                telemetry.addData("Direction Mode:" , "Forward Mode");
                imu.initialize(parameters);
            }

            angle = imu.getAngularOrientation().firstAngle;

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
            if(gamepad1.right_trigger < 0.5)
            {
                telemetry.addData("Status", "trigger off");
                coefficient=coefficient;
            }
            else
            {
                telemetry.addData("Status", "trigger on");
                coefficient=1;
            }

            telemetry.addData("Front Left Power", frontleftPower*coefficient);
            backrightDrive.setPower(backrightPower * coefficient);
            backleftDrive.setPower(backleftPower * coefficient);
            frontrightDrive.setPower(frontrightPower * coefficient);
            frontleftDrive.setPower(frontleftPower * coefficient);

            telemetry.update();
        }
    }


}
