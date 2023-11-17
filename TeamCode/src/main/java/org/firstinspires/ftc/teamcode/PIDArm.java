package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.InverseKinematics.inverseKinematics;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.utils.PIDController;

@TeleOp(name = "pidarmtesting")
public class PIDArm extends LinearOpMode {
    DcMotor arm1;
    DcMotor arm2;

    float finX = 20;
    float finY = 20;
    float armLength1 = 21.6f;
    float armLength2 = 31.2f;
    float spinCount1 = 0;
    float spinCount2 = 0;
    float lastArmAngle1 = 0;
    float lastArmAngle2 = 0;
    PIDController pidController1 = new PIDController(0.6f, 1.2f, 0.075f);
    PIDController pidController2 = new PIDController(0.6f, 1.2f, 0.075f);

    @Override
    public void runOpMode() throws InterruptedException {
        arm1 = hardwareMap.get(DcMotor.class, "baseArm");
        arm2 = hardwareMap.get(DcMotor.class, "floatingArm");

        arm1.setDirection(DcMotor.Direction.FORWARD);
        arm2.setDirection(DcMotor.Direction.FORWARD);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setTargetPosition(0);
        arm2.setTargetPosition(0);

        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm1.setPower(0.3);
        arm2.setPower(0.1);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            finY += y * 0.3f;
            finX += x * 0.3f;

            boolean isTriggered = false;
            double[] angles = inverseKinematics(armLength1, armLength2, finX, finY, isTriggered);

            float arm1Angle = (float) angles[0];
            float arm2Angle = (float) angles[1];

            if (lastArmAngle1 - arm1Angle > 300) {
                spinCount1++;
            } else if (lastArmAngle1 - arm1Angle < -300) {
                spinCount1--;
            }
            if (lastArmAngle2 - arm2Angle > 300) {
                spinCount2++;
            } else if (lastArmAngle2 - arm2Angle < -300) {
                spinCount2--;
            }

            float trueAngle1 = arm1Angle + spinCount1 * 360;
            float trueAngle2 = arm2Angle + spinCount2 * 360;

            lastArmAngle1 = arm1Angle;
            lastArmAngle2 = arm2Angle;
//
//            float vel1 = pidController1.update(trueAngle1 - finAngle1, dt);
//            float vel2 = pidController2.update(trueAngle2 - finAngle2, dt);
//
//            arm1.setTargetPosition((int) (finAngle1 * 14.45277777f));
//            arm2.setTargetPosition((int) (finAngle2 * -3.33333333333 + 612));
//
//            telemetry.addData("arm1: ", finAngle1);
//            telemetry.addData("arm2: ", finAngle2);
//
//
//            telemetry.addData("x: ", finX);
//            telemetry.addData("y: ", finY);
//            telemetry.addData("1e: ", arm1.getCurrentPosition());
//            telemetry.addData("2e: ", arm2.getCurrentPosition());

            telemetry.update();

        }

    }

}