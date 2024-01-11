package org.firstinspires.ftc.teamcode.configuration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test")
public class ServoTest extends LinearOpMode {
    Servo servo;

    double pos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();

        while (opModeIsActive()) {

            pos += gamepad1.left_stick_x * 0.05;
            if (gamepad1.dpad_up) {
                pos = 0;
            }
            if (gamepad1.dpad_down) {
                pos = 1;
            }

            servo.setPosition(pos);
            telemetry.addData("servo pos", pos);
            telemetry.update();

        }

    }

}
