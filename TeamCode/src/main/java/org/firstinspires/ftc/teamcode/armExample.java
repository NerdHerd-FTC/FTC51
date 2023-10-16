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

// Basic controls for arm and servo

@TeleOp(name = "Arm Example", group = "Examples")
public class armExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // initialize motors
        DcMotor slideMotor = hardwareMap.dcMotor.get("motorSlide");
        Servo armTopServo = hardwareMap.servo.get("armTopServo");
        DcMotor armRotateMotor = hardwareMap.dcMotor.get("armRotateMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        // set directions of motors
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armTopServo.setDirection(Servo.Direction.FORWARD);
        armRotateMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Variables initialized");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("RT - Extend Arm");
        telemetry.addLine("LT - Retract Arm");
        telemetry.addLine("RB - Rotate Arm Servo to Forward Position (150 Deg)");
        telemetry.addLine("LB - Rotate Arm Servo to 0 Degrees");
        telemetry.addLine("D-Pad Up - Rotate Arm Body Forwards");
        telemetry.addLine("D-Pad Down - Rotate Arm Body Backwards");
        telemetry.addLine("X button - Toggle intake on/off");
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
            // The position goes from 0 to 1
            // This represents the fraction of 300 degrees the motor should be at
            // eg. 0.5 would be 150 degrees & 0.1 would be 30.
            if (gamepad1.right_bumper){
                servoPosition=0.5; // 150 degrees
            } else if (gamepad1.left_bumper) {
                servoPosition=0; // 0 degrees
            }


            // controls to rotate the whole arm up and down (forwards and backwards)
            if (gamepad1.dpad_up) {
                armRotateMotor.setPower(0.1); //makes the arm motors rotate forwards slowly
                telemetry.addLine("Moving arm forward");
            } else if (gamepad1.dpad_down) {
                armRotateMotor.setPower(-0.1); //makes the arm motors rotate backwards slowly
                telemetry.addLine("Moving arm backward");
            } else {
                telemetry.addLine("Arm isn't moving");
            }

            if (gamepad1.x) {
                intakeMotor.setPower(1-intakeMotor.getPower());
                telemetry.addLine("Intake is moving");
            } else {
                telemetry.addLine("Intake is stopped");
            }

            slideMotor.setPower(rTrigger-lTrigger+0.05); // move slide motor
            // the 0.05 is to counteract gravity
            telemetry.addData("Current arm motion:",rTrigger-lTrigger);

            armTopServo.setPosition(servoPosition); // set servo position
            telemetry.addData("Arm Servo Position", servoPosition);

            telemetry.update();
        }
    }
}
