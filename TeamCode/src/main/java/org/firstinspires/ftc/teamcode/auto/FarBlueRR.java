package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.RobotMovement;
import org.firstinspires.ftc.teamcode.utils.RobotPositionAccessor;
import org.firstinspires.ftc.teamcode.utils.RobotPositions;

@Autonomous(name = "Far Blue")
public class FarBlueRR extends OpMode {
    enum States {
        TRY1,
        PLACE1,
        PLACE2,
        TRY2
    }

    States state = null;
    Servo claw;
    Servo clawHinge;
    DcMotor arm1;
    DcMotor arm2;
    double[] armPosition = { 0, 0, 0 };
    DistanceSensor ds;
    SampleMecanumDrive drive;
    double min = 99999999;
    double min2 = 99999999;

    Trajectory trajectory;
    Trajectory PLACE_PIXEL1;
    Trajectory trajectory2;

    @Override
    public void init() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        arm1 = hardwareMap.get(DcMotor.class, "baseArm");
        arm2 = hardwareMap.get(DcMotor.class, "floatingArm");
        arm1.setPower(0.3);
        arm2.setPower(0.1);
        arm1.setTargetPosition(arm1Position);
        arm2.setTargetPosition(arm2Position);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "claw");
        clawHinge = hardwareMap.get(Servo.class, "clawHinge");

        claw.setPosition(0.38);



        Trajectory trajectory = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-10, -24), Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    state = States.TRY1;
                })
                .addDisplacementMarker(() -> {
                    if (min <= 10) {
                        drive.followTrajectoryAsync(PLACE_PIXEL11);
                    } else {
                        drive.followTrajectoryAsync(trajectory2);
                    }
                })
                .build();

        Trajectory PLACE_PIXEL1 = drive.trajectoryBuilder(trajectory.end())
                .strafeRight(10)
                .addDisplacementMarker(() -> {
                    state = States.PLACE_PIXEL1;
                })
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory.end())
                .lineToLinearHeading()
                .build();
    }

    @Override
    public void loop() {
        if (state == States.TRY1) {
            min = Math.min(ds.getDistance(DistanceUnit.CM), min);
        } else if (state == States.TRY2) {
            min2 = Math.min(ds.getDistance(DistanceUnit.CM), min2);
        } else if (state == PLACE_PIXEL1) {
            armPosition = RobotPositionAccessor.getPosition(RobotPositions.PICKUP);

            if (!arm1.isBusy() && !arm2.isBusy()) {
                claw.setPosition(0.65);
            }
        }

        arm1.setTargetPosition(armPosition[0]);
        arm2.setTargetPosition(armPosition[1]);
        clawHinge.setPosition(armPosition[2]);

        telemetry.addData("Min", min);
        telemetry.addData("Min2", min2);
        drive.update();
        telemetry.update();
    }

    //
    // waitForStart();
    //
    // RobotMovement.right(510);
    // sleep(1500);
    // RobotMovement.forward(1180);
    // clawHinge.setPosition(0.65f);
    // sleep(1000);
    // RobotMovement.forward(1500);

    // double min = 999999;
    // while (backleftDrive.isBusy() || frontrightDrive.isBusy()) {
    // double distance = ds.getDistance(DistanceUnit.CM);
    // min = Math.min(distance, min);
    // telemetry.addData("Distance", distance);
    // telemetry.addData("Min", min);
    // telemetry.update();
    // }
    //
    // sleep(10);
    // RobotMovement.backward(700);
    // sleep(2000);
    // RobotMovement.turnLeft(900);
    // sleep(1500);
    // RobotMovement.forward(300);
    //
    // double min2 = 999999;
    // while (backleftDrive.isBusy() || frontrightDrive.isBusy()) {
    // double distance = ds.getDistance(DistanceUnit.CM);
    // min2 = Math.min(distance, min2);
    // telemetry.addData("Distance", distance);
    // telemetry.addData("Min", min);
    // telemetry.addData("Min2", min2);
    // telemetry.update();
    // }
    //
    // sleep(10);
    // RobotMovement.forward(200);
    // sleep(1000);
    // RobotMovement.backward(500);
    // sleep(1000);
    // RobotMovement.turnRight(900);
    // clawHinge.setPosition(0);
    // sleep(1500);
    // RobotMovement.backward(1890);
    // sleep(2500);
    // RobotMovement.left(510);
    // sleep(1000);
    //
    // int line = 1;
    // if (min <= 10) {
    // line = 3;
    // } else if (min2 <= 18) {
    // line = 2;
    // }
    //
    // int amount = 1280;
    // RobotMovement.forward(1380);
    //
    // sleep(1200);
    //
    //
    // if (line == 1 || line == 3) {
    // RobotMovement.forward(200);
    // sleep(500);
    // if (line == 1) {
    // RobotMovement.turnLeft(900);
    // sleep(1000);
    // RobotMovement.backward(20);
    // sleep(500);
    // } else if (line == 3) {
    // RobotMovement.turnRight(880);
    // sleep(1000);
    //
    // }
    // }
    //
    // arm1.setPower(0.3);
    // arm2.setPower(0.1);
    // arm1.setTargetPosition(1504);
    // arm2.setTargetPosition(840);
    // arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //
    // sleep(4000);
    // clawHinge.setPosition(0.65);
    //
    // sleep(1000);
    //
    // claw.setPosition(0.1f);
    //
    // sleep(1000);
    //
    // arm1.setPower(0.3);
    // arm2.setPower(0.1);
    // arm1.setTargetPosition(2357);
    // arm2.setTargetPosition(640);
    // arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // clawHinge.setPosition(0.65);
    //
    // sleep(3000);
    //
    // claw.setPosition(0.1f);
    //
    // sleep(1000);

    // if (line == 1) {
    // RobotMovement.turnRight(900);
    // sleep(1000);
    // } else if (line == 3) {
    // RobotMovement.turnLeft(900);
    // sleep(1000);
    // } else {
    // RobotMovement.backward(200);
    // sleep(500);
    // }
}}