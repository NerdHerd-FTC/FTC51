package org.firstinspires.ftc.teamcode.Examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Example", group = "Examples")
public class servoExample extends LinearOpMode {
    @Override
    public void runOpMode(){

        Servo servoOne = hardwareMap.get(Servo.class, "armTopServo");

        telemetry.addLine("initialized");

        telemetry.update();
        waitForStart();

        while (opModeIsActive()){

            double stickX = gamepad1.right_stick_x;

            servoOne.setPosition(stickX);

            telemetry.addData("Stick X", stickX+1);
            telemetry.update();
        }
    }
}
