package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="spikemarktest", group = "Tests")
public class spikeMarkTest extends autoExample {
    @Override
    public void runOpMode() throws InterruptedException{

        // Set motor directions
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset the encoder
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // fix drivetrain motor directions
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE); // IDK MAN, WE NEED TO TEST
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flMotor.setTargetPosition(0);
        brMotor.setTargetPosition(0);
        slideMotor.setTargetPosition(0);

        flMotor.setPower(1);
        brMotor.setPower(1);
        slideMotor.setPower(1);

        // Sets motors into go to position mode
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Switches from setting power to moving to a position
        
        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.y){
                positionCenter();
            }
            if (gamepad1.x){
                positionLeft();
            }
            if (gamepad1.b){
                positionRight();
            }
            telemetry.update();
        }
    }

    public void positionCenter() {
        driveFunc(100);
    }

    public void positionRight(){
        strafe(100,1);
    }
    public void positionLeft(){
        strafe(100,-1);
    }
}
