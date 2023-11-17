package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

@TeleOp(name = "basic arm")
public class ArmTesting2 extends LinearOpMode {
    DcMotor arm1;
    DcMotor arm2;

    float finX = 0;//-208;
    float finY = 0;//-1800;
    float armLength1 = 8.5f;
    float armLength2 = 18;

    float clawHingePosition = 0;

    Servo clawHinge;

    @Override
    public void runOpMode() throws InterruptedException {
        arm1 = hardwareMap.get(DcMotor.class, "baseArm");
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setTargetPosition((int) finY);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1.setPower(0.5);

        arm2 = hardwareMap.get(DcMotor.class, "floatingArm");
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setTargetPosition((int) finX);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setPower(0.5);

        clawHinge = hardwareMap.get(Servo.class, "clawHinge");

        waitForStart();

        while (opModeIsActive()) {
            clawHingePosition += gamepad2.right_stick_y * 0.005;
//            float clawHingeUpperBound = 1;
//            float clawHingeLowerBound = 0.66f;
//            if (clawHingePosition >= clawHingeUpperBound) {
//                clawHingePosition = clawHingeUpperBound;
//            } else if (clawHingePohdsition <= clawHingeLowerBound) {
//                clawHingePosition = clawHingeLowerBound;
//            }
            clawHinge.setPosition(clawHingePosition);
            telemetry.addData("claw hinge position", clawHingePosition);

            double y = gamepad1.left_stick_y;
            double x = gamepad1.right_stick_y;
            finY += y * 0.5f;
            finX += x * 0.5f;
            arm1.setTargetPosition((int) (finY * 14.45277777f));
            arm2.setTargetPosition((int) (finX * -3.33333333333 + 612));
            telemetry.addData("Arm Position: ", finY);
            telemetry.addData("Arm2 Position: ", finX);
            telemetry.addData("Arm Encoder: ", arm1.getCurrentPosition());
            telemetry.addData("Arm2 Position: ", arm2.getCurrentPosition());

            telemetry.update();



        }
    }

}