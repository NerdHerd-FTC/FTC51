package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Auto Example", group = "Examples")
public class autoExample extends LinearOpMode {

    // Immutable

    @Override
    public void runOpMode() throws InterruptedException {
        // Define motor variables
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");
        DcMotor slideMotor = hardwareMap.dcMotor.get("motorSlide");

        Servo armTopServo = hardwareMap.servo.get("armTopServo");
        DcMotor armRotateMotor = hardwareMap.dcMotor.get("armRotateMotor"); // baned 😮‍💨
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo droneServo = hardwareMap.servo.get("droneServo");

        telemetry.addLine("Started");
        telemetry.update();

        // Set motor directions
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addLine("directions set");
        telemetry.update();

        // Reset the encoder
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addLine("mode set");
        telemetry.update();

        // fix drivetrain motor directions
//        frMotor.setDirection(DcMotorSimple.Direction.REVERSE); // IDK MAN, WE NEED TO TEST
//        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        telemetry.addLine("update direction 2");
//        telemetry.update();

        flMotor.setTargetPosition(0);
        brMotor.setTargetPosition(0);
        slideMotor.setTargetPosition(0);
        telemetry.addLine("positions set");
        telemetry.update();

        flMotor.setPower(1);
        brMotor.setPower(1);
        slideMotor.setPower(1);
        telemetry.addLine("power set");
        telemetry.update();

        // Sets motors into go to position mode
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Switches from setting power to moving to a position
        telemetry.addLine("modes set");
        telemetry.update();

        waitForStart();

        // Move robot forward 150 millimeters
        telemetry.addLine("starting drive");
        telemetry.update();
        driveFunc(150,frMotor,brMotor,flMotor,blMotor);
        telemetry.addLine("drive has finished");
        telemetry.update();

        //strafe(100,1);
    }

    //537.7 Pulses per Rotation
    //PROBABLY 2148 counts per rotation
    //Wheels are 96mm diameter
    final double countsPerMM = 2148 / (96 * Math.PI);
    // Used to calculate the amount to move each motor

    public void driveFunc(double distance, DcMotor frMotor, DcMotor brMotor, DcMotor flMotor, DcMotor blMotor){
        telemetry.addLine("function ran");
        telemetry.update();
        // We only have a few encoder ports, so manually move the two other motors

        frMotor.setPower(Math.signum(distance));
        blMotor.setPower(Math.signum(distance));
        telemetry.addLine("power set");
        telemetry.update();

        // Moves the robot the distance forward
        // Distance is in millimeters

        flMotor.setTargetPosition(flMotor.getCurrentPosition() + (int) (distance * countsPerMM)); // Tell motors to move
        brMotor.setTargetPosition(brMotor.getCurrentPosition() + (int) (distance * countsPerMM));
        telemetry.addLine("target position set");
        telemetry.update();


        telemetry.addLine("waiting...");
        telemetry.update();
        // Wait for motors to stop moving
        while (flMotor.isBusy() || brMotor.isBusy()) {;} // busy loop; we need to fix later
        // maybe never if it works fine
        telemetry.addLine("finished");
        telemetry.update();

        frMotor.setPower(0); // Stop motors
        blMotor.setPower(0);
        telemetry.addLine("function finished");
        telemetry.update();
    }

    public void strafe(double distance, double direction, DcMotor frMotor, DcMotor brMotor, DcMotor flMotor, DcMotor blMotor){

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

    public void rotate(double distance, DcMotor frMotor, DcMotor brMotor, DcMotor flMotor, DcMotor blMotor) {
        // Moves the robot the distance forward
        // Distance is in millimeters clockwise

        flMotor.setTargetPosition(flMotor.getCurrentPosition() + (int) (distance * countsPerMM)); // Tell motors to move
        brMotor.setTargetPosition(brMotor.getCurrentPosition() - (int) (distance * countsPerMM));

        // We only have a few encoder ports, so manually move the two other motors
        frMotor.setPower(-Math.signum(distance));
        blMotor.setPower(Math.signum(distance));

        // Wait for motors to stop moving
        while (flMotor.isBusy() || brMotor.isBusy()) {;} // busy loop; we need to fix later
        // maybe never if it works fine

        frMotor.setPower(0); // Stop motors
        blMotor.setPower(0);
    }

}