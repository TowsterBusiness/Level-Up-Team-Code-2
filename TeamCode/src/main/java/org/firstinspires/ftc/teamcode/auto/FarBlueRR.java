package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.RobotStatics;

@Autonomous(name = "Far Blue RR")
public class FarBlueRR extends OpMode {
    enum States {
        PLACE_PIXEL,
    }

    States state = null;
    Servo claw;
    Servo clawHinge;
    DcMotor arm1;
    DcMotor arm2;
    double[] armPosition = {0, 0, 0};
    DistanceSensor ds;
    SampleMecanumDrive drive;
    double distance1 = Double.POSITIVE_INFINITY;
    double distance2 = Double.POSITIVE_INFINITY;

    TrajectorySequence trajectory;
    TrajectorySequence placePixel1;
    TrajectorySequence trajectory2;
    TrajectorySequence placePixel2;
    TrajectorySequence placePixel3;

    ElapsedTime et = new ElapsedTime();


    @Override
    public void init() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        arm1 = hardwareMap.get(DcMotor.class, "baseArm");
        arm2 = hardwareMap.get(DcMotor.class, "floatingArm");
        arm1.setPower(0.5);
        arm2.setPower(0.5);
        arm1.setTargetPosition((int) armPosition[0]);
        arm2.setTargetPosition((int) armPosition[1]);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "claw");
        clawHinge = hardwareMap.get(Servo.class, "clawHinge");
        ds = hardwareMap.get(DistanceSensor.class, "ds");

        claw.setPosition(0.38);

        trajectory = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-10, -24), Math.toRadians(270))
                .lineTo(new Vector2d(-10, -30))
                .waitSeconds(0.3)
                .addDisplacementMarker(() -> {
                    distance1 = Math.min(ds.getDistance(DistanceUnit.CM), distance1);
                    if (distance1 <= 10) {
                        drive.followTrajectorySequenceAsync(placePixel1);
                    } else {
                        drive.followTrajectorySequenceAsync(trajectory2);
                    }
                })
                .build();

        placePixel1 = drive.trajectorySequenceBuilder(trajectory.end())
                .lineTo(new Vector2d(-10, -24))
                .addDisplacementMarker(() -> {
                    placePixel();
                })
                .build();

        trajectory2 = drive.trajectorySequenceBuilder(trajectory.end())
                .splineToConstantHeading(new Vector2d(0, -30), Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .lineTo(new Vector2d(0, -36))
                .addDisplacementMarker(() -> {
                    distance2 = Math.min(ds.getDistance(DistanceUnit.CM), distance1);
                    if (distance1 <= 16) {
                        drive.followTrajectorySequenceAsync(placePixel2);
                    } else {
                        drive.followTrajectorySequenceAsync(placePixel3);
                    }
                })
                .build();

        placePixel2 = drive.trajectorySequenceBuilder(trajectory2.end())
                .lineTo(new Vector2d(-5, -34))
                .addDisplacementMarker(() -> {
                    placePixel();
                })
                .build();

        placePixel3 = drive.trajectorySequenceBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(0, -27, Math.toRadians(270)))
                .addDisplacementMarker(() -> {
                    et.reset();
                    placePixel();
                })
                .build();

        drive.followTrajectorySequenceAsync(trajectory);
    }

    public void placePixel() {
        state = States.PLACE_PIXEL;
        armPosition = RobotStatics.PLACE.clone();
    }

    @Override
    public void loop() {
        if (state == States.PLACE_PIXEL) {
            if (et.milliseconds() > 3000) {
                claw.setPosition(RobotStatics.clawOpenPos);
            }
        }

        telemetry.addData("et", et.milliseconds());

        arm1.setTargetPosition((int) armPosition[0]);
        arm2.setTargetPosition((int) armPosition[1]);
        clawHinge.setPosition(armPosition[2]);

        telemetry.addData("distance 1", distance1);
        telemetry.addData("distance 2", distance2);
        drive.update();
        telemetry.update();
    }

//    @Override
//    public void init() {
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        drive = new SampleMecanumDrive(hardwareMap);
//
//        arm1 = hardwareMap.get(DcMotor.class, "baseArm");
//        arm2 = hardwareMap.get(DcMotor.class, "floatingArm");
//        arm1.setPower(0.5);
//        arm2.setPower(0.5);
//        arm1.setTargetPosition((int) armPosition[0]);
//        arm2.setTargetPosition((int) armPosition[1]);
//        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        claw = hardwareMap.get(Servo.class, "claw");
//        clawHinge = hardwareMap.get(Servo.class, "clawHinge");
//        ds = hardwareMap.get(DistanceSensor.class, "ds");
//
//        claw.setPosition(0.38);
//
//
//
//        trajectory = drive.trajectorySequenceBuilder(new Pose2d())
//                .lineTo(new Vector2d(10, -24))
//                .lineTo(new Vector2d(10, -30))
//                .waitSeconds(0.3)
//                .addDisplacementMarker(() -> {
//                    min = Math.min(ds.getDistance(DistanceUnit.CM), min);
//                    if (min <= 10) {
//                        drive.followTrajectorySequenceAsync(placePixel1);
//                    } else {
//                        drive.followTrajectorySequenceAsync(trajectory2);
//                    }
//                })
//                .build();
//
//        placePixel1 = drive.trajectorySequenceBuilder(trajectory.end())
//                .strafeLeft(18)
//                .addDisplacementMarker(() -> {
//                    state = States.PLACE_PIXEL1;
//                })
//                .build();
//
//        trajectory2 = drive.trajectorySequenceBuilder(trajectory.end())
//                .lineTo(new Vector2d(0, -30))
//                .lineTo(new Vector2d(0, -34))
//                .addDisplacementMarker(() -> {
//                    min2 = Math.min(ds.getDistance(DistanceUnit.CM), min);
//                    if (min <= 10) {
//                        drive.followTrajectorySequenceAsync(placePixel2);
//                    } else {
//                        drive.followTrajectorySequenceAsync(placePixel3);
//                    }
//                })
//                .build();
//
//        placePixel2 = drive.trajectorySequenceBuilder(trajectory2.end())
//                .lineTo(new Vector2d(5, -34))
//                .addDisplacementMarker(() -> {
//                    state = States.PLACE_PIXEL1;
//                })
//                .build();
//
//        placePixel3 = drive.trajectorySequenceBuilder(trajectory2.end())
//                .lineToLinearHeading(new Pose2d(0, -30, Math.toRadians(-65)))
//                .addDisplacementMarker(() -> {
//                    et.reset();
//                    state = States.PLACE_PIXEL1;
//                })
//                .build();
//
//        drive.followTrajectorySequenceAsync(trajectory);
//    }
//
//    @Override
//    public void loop() {
//        if (state == States.PLACE_PIXEL1) {
//            telemetry.addData("hi", true);
//            armPosition[0] = 1275;
//            armPosition[1] = 3197;
//            armPosition[2] = 0.437f;
//            if (et.milliseconds() > 3000) {
//                claw.setPosition(0.1);
//            }
//        } else if (state == States.PLACE_PIXEL2) {
//            claw.setPosition(0.1);
//        }
//
//        telemetry.addData("arm pos 1", armPosition[0]);
//        telemetry.addData("arm pos 2", armPosition[1]);
//        telemetry.addData("arm pos 3", armPosition[2]);
//        telemetry.addData("et", et.milliseconds());
//
//        arm1.setTargetPosition((int) armPosition[0]);
//        arm2.setTargetPosition((int) armPosition[1]);
//        clawHinge.setPosition(armPosition[2]);
//
//        telemetry.addData("Min", min);
//        telemetry.addData("Min2", min2);
//        drive.update();
//        telemetry.update();
//    }

    //
//        waitForStart();
//
//        RobotMovement.right(510);
//        sleep(1500);
//        RobotMovement.forward(1180);
//        clawHinge.setPosition(0.65f);
//        sleep(1000);
//        RobotMovement.forward(1500);

//        double min = 999999;
//        while (backleftDrive.isBusy() || frontrightDrive.isBusy()) {
//            double distance = ds.getDistance(DistanceUnit.CM);
//            min = Math.min(distance, min);
//            telemetry.addData("Distance", distance);
//            telemetry.addData("Min", min);
//            telemetry.update();
//        }
//
//        sleep(10);
//        RobotMovement.backward(700);
//        sleep(2000);
//        RobotMovement.turnLeft(900);
//        sleep(1500);
//        RobotMovement.forward(300);
//
//        double min2 = 999999;
//        while (backleftDrive.isBusy() || frontrightDrive.isBusy()) {
//            double distance = ds.getDistance(DistanceUnit.CM);
//            min2 = Math.min(distance, min2);
//            telemetry.addData("Distance", distance);
//            telemetry.addData("Min", min);
//            telemetry.addData("Min2", min2);
//            telemetry.update();
//        }
//
//        sleep(10);
//        RobotMovement.forward(200);
//        sleep(1000);
//        RobotMovement.backward(500);
//        sleep(1000);
//        RobotMovement.turnRight(900);
//        clawHinge.setPosition(0);
//        sleep(1500);
//        RobotMovement.backward(1890);
//        sleep(2500);
//        RobotMovement.left(510);
//        sleep(1000);
//
//        int line = 1;
//        if (min <= 10) {
//            line = 3;
//        } else if (min2 <= 18) {
//            line = 2;
//        }
//
//        int amount = 1280;
//        RobotMovement.forward(1380);
//
//        sleep(1200);
//
//
//        if (line == 1 || line == 3) {
//            RobotMovement.forward(200);
//            sleep(500);
//            if (line == 1) {
//                RobotMovement.turnLeft(900);
//                sleep(1000);
//                RobotMovement.backward(20);
//                sleep(500);
//            } else if (line == 3) {
//                RobotMovement.turnRight(880);
//                sleep(1000);
//
//            }
//        }
//
//        arm1.setPower(0.3);
//        arm2.setPower(0.1);
//        arm1.setTargetPosition(1504);
//        arm2.setTargetPosition(840);
//        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        sleep(4000);
//        clawHinge.setPosition(0.65);
//
//        sleep(1000);
//
//        claw.setPosition(0.1f);
//
//        sleep(1000);
//
//        arm1.setPower(0.3);
//        arm2.setPower(0.1);
//        arm1.setTargetPosition(2357);
//        arm2.setTargetPosition(640);
//        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        clawHinge.setPosition(0.65);
//
//        sleep(3000);
//
//        claw.setPosition(0.1f);
//
//        sleep(1000);

//        if (line == 1) {
//            RobotMovement.turnRight(900);
//            sleep(1000);
//        } else if (line == 3) {
//            RobotMovement.turnLeft(900);
//            sleep(1000);
//        } else {
//            RobotMovement.backward(200);
//            sleep(500);
//        }
}