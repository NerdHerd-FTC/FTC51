package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Example")
public class servoExample extends LinearOpMode {
    @Override
    public void runOpMode(){

        Servo servoOne = hardwareMap.get(Servo.class, "servoOne");

        telemetry.addLine("initalized");

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
