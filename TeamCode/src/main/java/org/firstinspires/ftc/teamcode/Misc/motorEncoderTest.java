package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
public class motorEncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        //get all motors attached to robot
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");

        // Set motor directions
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset the encoder
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // fix drivetrain motor directions
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD); // IDK MAN, WE NEED TO TEST
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        frMotor.setTargetPosition(0);
        flMotor.setTargetPosition(0);
        brMotor.setTargetPosition(0);
        blMotor.setTargetPosition(0);

        frMotor.setPower(1); // IDK MAN, WE NEED TO TEST
        flMotor.setPower(1);
        brMotor.setPower(1);
        blMotor.setPower(1);

        // Sets motors into go to position mode
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Switches from setting power to moving to a position

        telemetry.addLine("Controls:");
        telemetry.addData("Front right motor","a");
        telemetry.addData("Back right motor","a");
        telemetry.addData("Front left motor","a");
        telemetry.addData("Back left motor","a");
        telemetry.addLine();
        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a){
                frMotor.setTargetPosition(100);
            }
            if(gamepad1.b){
                brMotor.setTargetPosition(100);
            }
            if(gamepad1.x){
                flMotor.setTargetPosition(100);
            }
            if(gamepad1.y){
                blMotor.setTargetPosition(100);
            }
            telemetry.addData("frMotor active",frMotor.isBusy());
            telemetry.addData("brMotor active",brMotor.isBusy());
            telemetry.addData("flMotor active",flMotor.isBusy());
            telemetry.addData("blMotor active",blMotor.isBusy());
            telemetry.update();
        }
    }
}
