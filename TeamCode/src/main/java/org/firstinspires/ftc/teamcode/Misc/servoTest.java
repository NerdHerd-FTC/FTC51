package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "servo test", group="Tests")
public class servoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo droneServo = hardwareMap.servo.get("droneServo");
        droneServo.setPosition(0);
        telemetry.addLine("ready");
        telemetry.update();
        waitForStart();
        droneServo.setPosition(0.3);
    }
}