package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Example - Button Control")
public class servoExampleButtonControl extends LinearOpMode {
    @Override
    public void runOpMode(){

        Servo armTopServo = hardwareMap.get(Servo.class, "servoOne");

        telemetry.addLine("initalized");

        telemetry.update();
        waitForStart();

        while (opModeIsActive()){

            //in our case, our servos have a range of 300 degrees
            //the position numbers are a fraction of these 300 degrees
            if (gamepad1.x) {
                armTopServo.setPosition(0);
            }

            if (gamepad1.y) {
                armTopServo.setPosition(0.3);
            }

            if (gamepad1.b) {
                armTopServo.setPosition(1);
            }

        }
    }
}
