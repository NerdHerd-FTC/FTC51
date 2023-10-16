package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Auto Example", group = "Examples")
public class autoExample extends LinearOpMode {
    // Define motor variables
    private final DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
    private final DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
    private final DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
    private final DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");
    private final DcMotor slideMotor = hardwareMap.dcMotor.get("motorSlide");
    // Immutable

    @Override
    public void runOpMode() throws InterruptedException {

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

        // Sets motors into go to position mode
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Switches from setting power to moving to a position

        waitForStart();

        // Move robot forward 150 millimeters
        driveFunc(150);

        telemetry.addLine("Drive has started"); // Test to see if multithreading works
        telemetry.update();
        // If the telemetry does not display until driving has finished, it don't work.

        strafe(100,1);
    }

    //537.7 Pulses per Rotation
    //PROBABLY 2148 counts per rotation
    //Wheels are 96mm diameter
    final double countsPerMM = 2148 / (96 * Math.PI);
    // Used to calculate the amount to move each motor

    public void driveFunc(double distance){
        // Moves the robot the distance forward
        // Distance is in millimeters

        flMotor.setTargetPosition(flMotor.getCurrentPosition() + (int) (distance * countsPerMM)); // Tell motors to move
        brMotor.setTargetPosition(brMotor.getCurrentPosition() + (int) (distance * countsPerMM));

        // We only have a few encoder ports, so manually move the two other motors
        frMotor.setPower(1);
        blMotor.setPower(1);

        // Wait for motors to stop moving
        while (flMotor.isBusy() || brMotor.isBusy()) {;} // busy loop; we need to fix later
        // maybe never if it works fine

        frMotor.setPower(0); // Stop motors
        blMotor.setPower(0);
    }

    public void strafe(double distance, double direction){
        flMotor.setTargetPosition(flMotor.getCurrentPosition() + (int) ((distance * countsPerMM) * direction));
        brMotor.setTargetPosition(brMotor.getCurrentPosition() + (int) ((distance * countsPerMM) * direction));

        // We only have a few encoder ports, so manually move the two other motors
        frMotor.setPower(-direction);
        blMotor.setPower(-direction);

        // Wait for motors to stop moving
        while (flMotor.isBusy() || brMotor.isBusy()) {;} // busy loop; we need to fix later
        // maybe never if it works fine

        frMotor.setPower(0); // Stop motors
        blMotor.setPower(0);

        // REMOVE AFTER TESTING THREADS
        telemetry.addLine("Strafe has finished");
        telemetry.update();

    }

}
