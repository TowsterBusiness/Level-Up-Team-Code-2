package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.InverseKinematics;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

@TeleOp(name = "armtesting")
public class ArmTesting extends LinearOpMode {
    DcMotor arm1;
    DcMotor arm2;

    float finX = 20;
    float finY = 20;
    float armLength1 = 21.6f;
    float armLength2 = 31.2f;

    float spinCount1 = 0;
    float spinCount2 = 0;
    float pastArmAngle1 = 0;
    float pastArmAngle2 = 0;

    PIDController pid1 = new PIDController(0.8f, 0f, 0);
    PIDController pid2 = new PIDController(0.8f, 0f, 0);

    ElapsedTime et = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        arm1 = hardwareMap.get(DcMotor.class, "baseArm");
        arm2 = hardwareMap.get(DcMotor.class, "floatingArm");

        arm1.setDirection(DcMotor.Direction.FORWARD);
        arm2.setDirection(DcMotor.Direction.FORWARD);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm1.setPower(0);
        arm2.setPower(0);

        waitForStart();

        et.reset();

        while (opModeIsActive()) {
            float dt = (float) et.milliseconds();
            et.reset();
            telemetry.addData("dt", dt);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            finY += y * 0.3f;
            finX += x * 0.3f;

            double[] positions = InverseKinematics.inverseKinematics(armLength1, armLength2, finX, finY, false);

            float arm3Angle = (float) positions[0];
            float arm4Angle = (float) positions[1];

            if (arm3Angle > 339 || arm3Angle < 201) {

                if (pastArmAngle1 - arm3Angle > 300) {
                    spinCount1++;
                } else if (pastArmAngle1 - arm3Angle < -300) {
                    spinCount1--;
                }
                float finAngle1 = arm3Angle + spinCount1 * 360;

                if (pastArmAngle2 - arm4Angle > 300) {
                    spinCount2++;
                } else if (pastArmAngle2 - arm4Angle < -300) {
                    spinCount2--;
                }
                float finAngle2 = arm4Angle + spinCount2 * 360;

                pastArmAngle1 = arm3Angle;
                pastArmAngle2 = arm4Angle;

                float v1 = pid1.update(arm1.getCurrentPosition() / 14.45277777f - finAngle1, dt);
                float v2 = pid2.update((arm2.getCurrentPosition() - 612) / -3.33333333f - finAngle2, dt);

                // allows breaking if there isn't muh velocity applied
                float vf1 = Math.abs(v1 / -17) < 1E-2 ? 0 : v1 / -17;
                float vf2 = Math.abs(v1 / 204) < 1E-2 ? 0 : v1 / 204;
                arm1.setPower(vf1);
                arm2.setPower(vf2);

                telemetry.addData("v1: ", vf1);
                telemetry.addData("v2: ", vf2);
                telemetry.addData("arm1: ", finAngle1);
                telemetry.addData("arm2: ", finAngle2);
            } else {
                arm1.setPower(0);
                arm2.setPower(0);
            }

            telemetry.addData("x: ", finX);
            telemetry.addData("y: ", finY);
            telemetry.addData("1e: ", arm1.getCurrentPosition());
            telemetry.addData("2e: ", arm2.getCurrentPosition());

            telemetry.update();

        }

    }

}