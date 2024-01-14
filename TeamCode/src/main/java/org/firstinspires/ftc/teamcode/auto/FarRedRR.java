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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.utils.RobotMovement;
import org.firstinspires.ftc.teamcode.utils.RobotPositionAccessor;

@Autonomous(name = "Far Red RR")
public class FarRedRR extends OpMode {
    enum States {
        TRY1,
        PLACE_PIXEL1,
        PLACE_PIXEL2,
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
        arm1.setPower(0.3);
        arm2.setPower(0.3);
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



        trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(-10, -24))
                .lineTo(new Vector2d(-10, -30))
                .waitSeconds(0.3)
                .addDisplacementMarker(() -> {
                    min = Math.min(ds.getDistance(DistanceUnit.CM), min);
                    if (min <= 10) {
                        drive.followTrajectorySequenceAsync(placePixel1);
                    } else {
                        drive.followTrajectorySequenceAsync(trajectory2);
                    }
                })
                .build();

        placePixel1 = drive.trajectorySequenceBuilder(trajectory.end())
                .strafeLeft(18)
                .addDisplacementMarker(()    -> {
                    state = States.PLACE_PIXEL1;
                })
                .waitSeconds(3)
                .addDisplacementMarker(()    -> {
                    state = States.PLACE_PIXEL2;
                })
                .build();

        trajectory2 = drive.trajectorySequenceBuilder(trajectory.end())
                .lineTo(new Vector2d(0, -30))
                .lineTo(new Vector2d(0, -34))
                .addDisplacementMarker(() -> {
                    min2 = Math.min(ds.getDistance(DistanceUnit.CM), min);
                    if (min <= 10) {
                        drive.followTrajectorySequenceAsync(placePixel2);
                    } else {
                        drive.followTrajectorySequenceAsync(placePixel3);
                    }
                })
                .build();

        placePixel2 = drive.trajectorySequenceBuilder(trajectory2.end())
                .lineTo(new Vector2d(5, -34))
                .addDisplacementMarker(() -> {
                    state = States.PLACE_PIXEL1;
                })
                .waitSeconds(3)
                .addDisplacementMarker(()    -> {
                    state = States.PLACE_PIXEL2;
                })
                .build();

        placePixel3 = drive.trajectorySequenceBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(0, -30, Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    state = States.PLACE_PIXEL1;
                })
                .waitSeconds(3)
                .addDisplacementMarker(()    -> {
                    state = States.PLACE_PIXEL2;
                })
                .build();

        drive.followTrajectorySequenceAsync(trajectory);
    }

    @Override
    public void loop() {
        if (state == States.PLACE_PIXEL1) {
            armPosition[0]=1275;
            armPosition[2]=3197;
            armPosition[3]=0.437f;
        } else if (state == States.PLACE_PIXEL2) {
            claw.setPosition(0.1);
        }

        arm1.setTargetPosition((int) armPosition[0]);
        arm2.setTargetPosition((int) armPosition[1]);
        clawHinge.setPosition(armPosition[2]);

        telemetry.addData("Min", min);
        telemetry.addData("Min2", min2);
        drive.update();
        telemetry.update();
    }
}