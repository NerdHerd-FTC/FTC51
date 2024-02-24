package org.firstinspires.ftc.teamcode.Examples.Servo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Example - Button Control", group = "Examples")
@Disabled
public class servoExampleButtonControl extends LinearOpMode {
    @Override
    public void runOpMode(){

        Servo armTopServo = hardwareMap.get(Servo.class, "armTopServo");

        telemetry.addLine("initialized");

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
