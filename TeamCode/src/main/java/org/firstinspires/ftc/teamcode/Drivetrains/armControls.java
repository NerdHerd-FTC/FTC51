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

    public void armControls(DcMotor slideMotor, Servo armTopServo, DcMotor armRotateMotor, DcMotor intakeMotor, Servo droneServo) {
        // initialize variables


        double gravityOffset=0.001;

        // 0.063 * 1/300
        double rotationFactor=0.063/300;

        //triggers for arm extending
        double rTrigger = gamepad2.right_trigger;
        double lTrigger = gamepad2.left_trigger;

        if (gamepad1.a) { // Toggle intake motor on/off
            if (!intakeButtonPressed) {
                intakeMotor.setPower(1 - intakeMotor.getPower());
                intakeButtonPressed = true;
            }
        } else {
            intakeButtonPressed = false;
        }
        telemetry.addData("Intake power",intakeMotor.getPower());

        // controls to rotate the whole arm up and down (forwards and backwards)
        // only changes position when the motor isn't busy, (hopefully) making controls more precise
        // 5700.4 counts per revolution
        // 6.3 degrees per button press
        if (gamepad2.dpad_up && !armRotateMotor.isBusy()) {
            armRotateMotor.setTargetPosition(armRotateMotor.getCurrentPosition() + 100); //makes the arm motors rotate forwards slowly
        } else if (gamepad2.dpad_down && !armRotateMotor.isBusy()) {
            armRotateMotor.setTargetPosition(armRotateMotor.getCurrentPosition() - 100); //makes the arm motors rotate backwards slowly
        }

        // Servos have a range of 300 degrees
        double armServoPosition = 0.95 + (armRotateMotor.getTargetPosition()*rotationFactor);
        // Calculate what position to rotate arm to
        // 0.75 is the base
        // Then add the current arm position, times the rotation factor
        // rotation factor = 0.063*0.0033333...
        // 0.063 is about how much a single degree is in relation to the encoder output
        // 0.003333... is about how much a single degree is in relation to the servo's range
        if (gamepad2.right_bumper){
            armTopServo.setPosition(armServoPosition);
        } else if (gamepad2.left_bumper) {
            armTopServo.setPosition(0.365);
        }

        telemetry.addData("armPosition",armRotateMotor.getCurrentPosition());


        //moves the drone servo to the launch position
        if (gamepad1.back) {
            droneServo.setPosition(1);
            droneLaunched=true;
        }
        telemetry.addData("Drone Launched",droneLaunched);

        if (slideMotor.getCurrentPosition()+rTrigger-lTrigger<1400){ // detect if upwards movement will go over
            slideMotor.setPower(rTrigger-lTrigger+gravityOffset); // move slide motor
        } else{
            slideMotor.setPower(-lTrigger+gravityOffset); // move slide motor only down
        }
        telemetry.addData("Slide position",slideMotor.getCurrentPosition());

    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("This opmode provides arm controls to other opmodes and doesn't do anything alone.");
        telemetry.update();
        waitForStart();
    }
}
