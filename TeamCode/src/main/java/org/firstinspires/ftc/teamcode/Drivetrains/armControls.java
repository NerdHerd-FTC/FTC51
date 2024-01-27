package org.firstinspires.ftc.teamcode.Drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="arm controls")
@Disabled
public class armControls extends LinearOpMode {
    private boolean droneLaunched = false;
    private boolean intakeButtonPressed = false;
    int direction = 1;

    // initialize variables
    double gravityOffset=0.001;

    //0-1 with 0 being 0 degrees and 1 being about 300 degrees
    double servoReleasePosition=.3;
    double servoLoadPosition=0;

    public void armControls(DcMotor slideMotorR, DcMotor slideMotorL, Servo armTopServoR, Servo armTopServoL, DcMotor intakeMotor, Servo droneServo, DcMotor hangMotor) {


        //triggers for arm extending
        double rTrigger = gamepad2.right_trigger;
        double lTrigger = gamepad2.left_trigger;

        if (gamepad1.b) {
            direction=-direction;
            intakeMotor.setPower(intakeMotor.getPower()*-1);
        }
        if (gamepad1.a) { // Toggle intake motor on/off
            if (!intakeButtonPressed) {
                intakeMotor.setPower((1 - Math.abs(intakeMotor.getPower()))*direction);
                intakeButtonPressed = true;
            }
        } else {
            intakeButtonPressed = false;
        }
        telemetry.addData("Intake power", Math.abs(intakeMotor.getPower()));
        telemetry.addData("Intake direction", (direction==1) ? "Forward" : "Backward");

        //Servos are 0-1 with a range of 300 degrees
        if (gamepad2.right_bumper){
            armTopServoR.setPosition(servoReleasePosition);
            armTopServoL.setPosition(servoReleasePosition);
        } else if (gamepad2.left_bumper) {
            armTopServoR.setPosition(servoLoadPosition);
            armTopServoL.setPosition(servoLoadPosition);
        }

        telemetry.addData("servoPosition",armTopServoR.getPosition());

        //moves the drone servo to the launch position
        if (gamepad1.back) {
            droneServo.setPosition(0.3);
            droneLaunched=true;
        }
        telemetry.addData("Drone Launched",droneLaunched);

        if (rTrigger>0 && (slideMotorR.getCurrentPosition()+rTrigger-lTrigger<1300 || slideMotorL.getCurrentPosition()+rTrigger-lTrigger<1400)){ // detect if upwards movement will go over
            slideMotorR.setPower(rTrigger-lTrigger+gravityOffset); // move slide motor
            slideMotorL.setPower(rTrigger-lTrigger+gravityOffset);
        } else if (lTrigger>0 && (slideMotorR.getCurrentPosition()-lTrigger>0 || slideMotorL.getCurrentPosition()-lTrigger>0)){
            slideMotorR.setPower(-lTrigger+gravityOffset); // move slide motor only down
            slideMotorL.setPower(-lTrigger+gravityOffset);
        } else{
            slideMotorR.setPower(gravityOffset);
            slideMotorL.setPower(gravityOffset);
        }
        telemetry.addData("Slide R position",slideMotorR.getCurrentPosition());
        telemetry.addData("Slide L position",slideMotorL.getCurrentPosition());

    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("This opmode provides arm controls to other opmodes and doesn't do anything alone.");
        telemetry.update();
        waitForStart();
    }
}
