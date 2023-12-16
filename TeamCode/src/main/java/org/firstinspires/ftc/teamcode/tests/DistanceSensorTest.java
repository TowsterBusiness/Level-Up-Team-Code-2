
package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "test auto")
public class DistanceSensorTest extends LinearOpMode {
    DistanceSensor ds;
    @Override
    public void runOpMode() throws InterruptedException {
        ds = hardwareMap.get(DistanceSensor.class, "ds");
        ds.getDistance(DistanceUnit.CM)
    }
}