package org.firstinspires.ftc.teamcode.configuration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test")
public class ServoTest extends LinearOpMode {
    Servo servo;

    Servo servo2;

    Servo servo3;

    Servo servo4;
    Servo servo5;

    DcMotor motor1;

    DcMotor motor2;
    double pos = 0;
    double pos2 = 0;

    double pos3 = 0;

    double pos4 = 0;
    double pos5 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "droneHinge");
        servo2 = hardwareMap.get(Servo.class, "claw2");
        servo3 = hardwareMap.get(Servo.class, "claw");//droneHinge");drone
        servo4 = hardwareMap.get(Servo.class, "droneSafety");
//        servo5 = hardwareMap.get(Servo.class, "claw2");

        waitForStart();

        while (opModeIsActive()) {

            pos += gamepad1.left_stick_x * 0.01;
            if (gamepad1.dpad_up) {
                pos = 0;
            }
            if (gamepad1.dpad_down) {
                pos = 1;
            }
            servo.setPosition(pos);
            telemetry.addData("servo pos", pos);

            pos2 += gamepad1.right_stick_x * 0.01;
            if (gamepad1.y) {
                pos2 = 0;
            }
            if (gamepad1.a) {
                pos2 = 1;
            }
            servo2.setPosition(pos2);
            telemetry.addData("servo pos 2", pos2);

            pos3+= gamepad2.left_stick_x * 0.01;
            if (gamepad2.dpad_up) {
                pos3 = 0;
            }
            if (gamepad2.dpad_down) {
                pos3 = 1;
            }
            servo3.setPosition(pos3);
            telemetry.addData("servo pos 3", pos3);

            pos4 += gamepad2.right_stick_x * 0.01;
            if (gamepad2.y) {
                pos4 = 0;
            }
            if (gamepad2.a) {
                pos4 = 1;
            }
            servo4.setPosition(pos4);
            telemetry.addData("servo pos 4", pos4);
            telemetry.update();





        }

    }

}
