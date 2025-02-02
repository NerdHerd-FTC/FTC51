package org.firstinspires.ftc.teamcode.Drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="arm controls")
@Disabled
public class armControls extends LinearOpMode {
    private boolean droneLaunched = false;
    private boolean intakeButtonPressed = false;
    private boolean directionButtonPressed = false;
    private boolean bucketDownButtonPressed = false;
    private boolean bucketUpButtonPressed = false;
    int direction = -1;

    // initialize variables
    double gravityOffset=0.0005;

    //0-1 with 0 being 0 degrees and 1 being about 300 degrees
    public static double servoLimitPosition = (double) 5 /7.5;
    public static double servoLoadPosition=0;
    //the denominator determines how many presses are needed to reach the release position.
    public static double servoStepAmount = (double) 1 /7.5;

    double intakeServoUp = 0.5;
    double intakeServoDown = 0;
    double intakeServoMUp = 0;
    double intakeServoMDown = 0.5;

    public String armControls(DcMotor slideMotorR, DcMotor slideMotorL, Servo armTopServoR, Servo armTopServoL, DcMotor intakeMotor, Servo droneServo, Servo intakeServoL, Servo intakeServoM, Servo intakeServoR, Gamepad gamepad1, Gamepad gamepad2) {
        String currentTelemetry = "";

        //triggers for arm extending
        double rTrigger = gamepad2.right_trigger;
        double lTrigger = gamepad2.left_trigger;

        if (gamepad1.b) { // toggle intake direction
            if (!directionButtonPressed) {
                direction = -direction;
                intakeMotor.setPower(intakeMotor.getPower() * -1);
                directionButtonPressed = true;
            }
        } else {
            directionButtonPressed = false;
        }
        if (gamepad1.a) { // Toggle intake motor on/off
            if (!intakeButtonPressed) {
                intakeMotor.setPower((1 - Math.abs(intakeMotor.getPower())) * direction);
                intakeButtonPressed = true;
            }
        } else {
            intakeButtonPressed = false;
        }
        currentTelemetry += "Intake power : " + Math.abs(intakeMotor.getPower());
        currentTelemetry += "\nIntake direction : " + ((direction == -1) ? "Forward" : "Backward");

        //Servos are 0-1 with a range of 300 degrees
        double currentBucketPosition = armTopServoR.getPosition();

        if (gamepad2.right_bumper) {
            if (currentBucketPosition < servoLimitPosition && !bucketUpButtonPressed) {
                armTopServoR.setPosition(currentBucketPosition + (servoStepAmount));
                armTopServoL.setPosition(currentBucketPosition + (servoStepAmount));
            }
            bucketUpButtonPressed = true;
        } else {
            bucketUpButtonPressed = false;
        }

        if (gamepad2.left_bumper) {
            if (currentBucketPosition > servoLoadPosition && !bucketDownButtonPressed) {
                armTopServoR.setPosition(currentBucketPosition - (servoStepAmount));
                armTopServoL.setPosition(currentBucketPosition - (servoStepAmount));
            }
            bucketDownButtonPressed = true;
        } else {
            bucketDownButtonPressed = false;
        }

        currentTelemetry += "\nArm Servo Position: " + currentBucketPosition;

        if (gamepad1.right_bumper){
            intakeServoL.setPosition(intakeServoDown);
        } else {
            intakeServoL.setPosition(intakeServoUp);
        }

        currentTelemetry += "\nLeft Intake Servo Position: " + intakeServoL.getPosition();

//        if (gamepad1.x){
//            intakeServoM.setPosition(intakeServoMDown);
//        } else {
//            intakeServoM.setPosition(intakeServoMUp);
//        }


        if (gamepad1.left_bumper){
            intakeServoR.setPosition(intakeServoDown);
        } else {
            intakeServoR.setPosition(intakeServoUp);
        }

        currentTelemetry += "\nRightIntake Servo Position: " + intakeServoR.getPosition();

        //moves the drone servo to the launch position
        if (gamepad1.back) {
            droneServo.setPosition(.6);
            droneLaunched=true;
        }
        currentTelemetry+= "\nDrone Launched"+ droneLaunched;

        if (rTrigger>0 && (slideMotorR.getCurrentPosition()+rTrigger-lTrigger<2200 || slideMotorL.getCurrentPosition()+rTrigger-lTrigger<1400)){ // detect if upwards movement will go over
            slideMotorR.setPower(rTrigger-lTrigger+gravityOffset); // move slide motor
            slideMotorL.setPower(rTrigger-lTrigger+gravityOffset);
        } else if (lTrigger>0 && (slideMotorR.getCurrentPosition()-lTrigger>0 || slideMotorL.getCurrentPosition()-lTrigger>0)){
            slideMotorR.setPower(-lTrigger+gravityOffset); // move slide motor only down
            slideMotorL.setPower(-lTrigger+gravityOffset);
        } else{
            slideMotorR.setPower(gravityOffset);
            slideMotorL.setPower(gravityOffset);
        }
        currentTelemetry+= "\nSlide R position"+ slideMotorR.getCurrentPosition();
        currentTelemetry+= "\nSlide L position"+ slideMotorL.getCurrentPosition();

        return currentTelemetry;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("This opmode provides arm controls to other opmodes and doesn't do anything alone.");
        telemetry.update();
        waitForStart();
    }
}
