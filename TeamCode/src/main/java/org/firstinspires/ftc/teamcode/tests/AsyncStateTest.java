package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "async state test")
public class AsyncStateTest extends OpMode {
    enum States {
        START,
        OBJECTIVE2
    }

    States state = States.START;
    SampleMecanumDrive drive;

    Trajectory trajectory1;
    Trajectory trajectory2;
    DistanceSensor ds;

    @Override
    public void init() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        ds = hardwareMap.get(DistanceSensor.class, "ds");

        trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .forward(50)
                .addDisplacementMarker(() -> {
                    state = States.OBJECTIVE2;
                    drive.followTrajectoryAsync(trajectory2);
                })
                .build();

        trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .strafeRight(10)
                .build();

        drive.followTrajectoryAsync(trajectory1);
    }

    @Override
    public void loop() {
        if (state == States.START) {
            // Start Async Commands
        } else if (state == States.OBJECTIVE2) {
            double distance = ds.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance", distance);

        }
        drive.update();
        telemetry.update();
    }
}
