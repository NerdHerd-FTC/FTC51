package org.firstinspires.ftc.teamcode;

//javadocs here: https://javadoc.io/doc/org.firstinspires.ftc
//ftc docs here: https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Drive train controls for mecanum drive
// Mecanum drive allows omnidirectional movement

@TeleOp(name = "Arm Example")
public class armExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor slideMotor = hardwareMap.dcMotor.get("motorSlide");
        Servo armTopServo = hardwareMap.servo.get("armTopServo");

        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armTopServo.setDirection(Servo.Direction.FORWARD);

        telemetry.addLine("Variables initialized");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("RT - Extend Arm");
        telemetry.addLine("LT - Retract Arm");
        telemetry.addLine("RB - Rotate Arm Servo Forwards");
        telemetry.addLine("X - Reset Arm Servo");
        telemetry.addLine();
        telemetry.addLine("Ready to start");
        telemetry.update();

        //waits for start of game
        waitForStart();

        double servoPosition = 0.3;

        while (opModeIsActive()) {

            //triggers for arm extending
            double rTrigger = gamepad1.right_trigger;
            double lTrigger = gamepad1.left_trigger;


            //in our case, our servos have a range of 300 degrees
            //the position numbers are a fraction of these 300 degrees
            if (gamepad1.right_bumper){
                servoPosition = servoPosition + 0.05;
            }

            if (gamepad1.x) {
                servoPosition = 0;
            }

            slideMotor.setPower(rTrigger-lTrigger);

            armTopServo.setPosition(servoPosition);
            telemetry.addData("Arm Servo Position", servoPosition);

            telemetry.update();
        }
    }
}
