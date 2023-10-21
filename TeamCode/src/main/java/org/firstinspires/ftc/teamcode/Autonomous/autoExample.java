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
        String[] telemetries = new String[10];
        for(int i = 0; i < telemetries.length; i++){
            telemetries[i] = "";
        }

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

        telemetries=print("Started",telemetries);

        // Set motor directions
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetries=print("directions set",telemetries);

        // Reset the encoder
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetries=print("mode set",telemetries);

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
        telemetries=print("positions set",telemetries);

        flMotor.setPower(1);
        brMotor.setPower(1);
        slideMotor.setPower(1);
        telemetries=print("power set",telemetries);

        // Sets motors into go to position mode
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Switches from setting power to moving to a position
        telemetries=print("modes set",telemetries);

        waitForStart();

        // Move robot forward 150 millimeters
        telemetries=print("starting drive",telemetries);
        driveFunc(150,frMotor,brMotor,flMotor,blMotor,telemetries);
        telemetries=print("drive has finished",telemetries);

        //strafe(100,1);
    }

    //537.7 Pulses per Rotation
    //PROBABLY 2148 counts per rotation
    //Wheels are 96mm diameter
    final double countsPerMM = 2148 / (96 * Math.PI);
    // Used to calculate the amount to move each motor

    public String[] driveFunc(double distance, DcMotor frMotor, DcMotor brMotor, DcMotor flMotor, DcMotor blMotor,String[] telemetries){
        telemetries=print("function ran",telemetries);
        // We only have a few encoder ports, so manually move the two other motors

        frMotor.setPower(Math.signum(distance));
        blMotor.setPower(Math.signum(distance));
        telemetries=print("power set",telemetries);

        // Moves the robot the distance forward
        // Distance is in millimeters

        flMotor.setTargetPosition(flMotor.getCurrentPosition() + (int) (distance * countsPerMM)); // Tell motors to move
        brMotor.setTargetPosition(brMotor.getCurrentPosition() + (int) (distance * countsPerMM));
        telemetries=print("target position set",telemetries);


        telemetries=print("waiting...",telemetries);
        // Wait for motors to stop moving
        while (flMotor.isBusy() || brMotor.isBusy()) {;} // busy loop; we need to fix later
        // maybe never if it works fine
        telemetries=print("finished",telemetries);

        frMotor.setPower(0); // Stop motors
        blMotor.setPower(0);
        telemetries=print("function finished",telemetries);

        return telemetries;
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

//    public String[] telemetries = new String[10];

    public String[] print(String text,String[] telemetries){
        // shift all elements
        for(int i = telemetries.length - 2; i >= 0; i--){
            telemetries[i + 1] = telemetries[i];
        }
        telemetries[0]=text;

        for(int i = 0;i<telemetries.length;i++){
            telemetry.addLine(telemetries[i]);
        }
        telemetry.update();

        return telemetries;
    }

}