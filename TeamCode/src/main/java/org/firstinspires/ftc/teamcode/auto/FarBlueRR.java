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
    Servo claw2;
    Servo clawHinge;
    DcMotor arm1;
    DcMotor arm2;
    double[] armPosition = RobotStatics.RESET.clone();
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
        claw2 = hardwareMap.get(Servo.class, "claw2");
        clawHinge = hardwareMap.get(Servo.class, "clawHinge");
        ds = hardwareMap.get(DistanceSensor.class, "ds");

        claw.setPosition(RobotStatics.clawClosedPos);
        claw2.setPosition(RobotStatics.claw2ClosedPos);
        clawHinge.setPosition(armPosition[2]);

        drive.setPoseEstimate(new Pose2d(-36, 61, Math.toRadians(0)));

        trajectory = drive.trajectorySequenceBuilder(new Pose2d(-36, 61, Math.toRadians(0)))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-44, 37), Math.toRadians(270))
                .lineTo(new Vector2d(-44, 27))
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
                .setTangent(90)
                .splineToConstantHeading(new Vector2d(-47.5, 43.5), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    placePixel();
                })
                .build();

        trajectory2 = drive.trajectorySequenceBuilder(trajectory.end())
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-39, 31), Math.toRadians(270))
                .lineTo(new Vector2d(-39, 15))
                .waitSeconds(3)
                .addDisplacementMarker(() -> {
                    distance2 = Math.min(ds.getDistance(DistanceUnit.CM), distance1);
                    if (distance2 <= 16) {
                        drive.followTrajectorySequenceAsync(placePixel2);
                    } else {
                        drive.followTrajectorySequenceAsync(placePixel3);
                    }
                })
                .build();

        placePixel2 = drive.trajectorySequenceBuilder(trajectory2.end())
                .setTangent(90)
                .lineTo(new Vector2d(-36, 30))
                .splineToConstantHeading(new Vector2d(-52, 21), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    placePixel();
                })
                .build();

        placePixel3 = drive.trajectorySequenceBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(-42.5, 22.5, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    placePixel();
                })
                .waitSeconds(1)
                .build();

        drive.followTrajectorySequenceAsync(trajectory);
    }

    public void placePixel() {
        et.reset();
        state = States.PLACE_PIXEL;
        armPosition = RobotStatics.PICKUP.clone();
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

        telemetry.addData("arm1 theo", armPosition[0]);
        telemetry.addData("arm2 theo", armPosition[1]);
        telemetry.addData("claw theo", armPosition[2]);

        telemetry.addData("distance 1", distance1);
        telemetry.addData("distance 2", distance2);
        drive.update();
        telemetry.update();
    }
}