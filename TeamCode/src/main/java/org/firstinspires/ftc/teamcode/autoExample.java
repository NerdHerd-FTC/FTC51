package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Auto Example")
public class autoExample extends LinearOpMode {
    private final DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
    private final DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
    private final DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
    private final DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");

    private final DcMotor slideMotor = hardwareMap.dcMotor.get("motorSlide");

    @Override
    public void runOpMode() throws InterruptedException {

        //reverse right side motors. reverse left side if goes backwards
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        //distance is in mm
        //create thread for async running
        Thread runDrive = new Thread(() -> driveFunc(150));
        runDrive.start();

        telemetry.addLine("wow it run it good");
        telemetry.update();
    }

    public void driveFunc(double distance){

        //537.7 Pulses per Rotation
        //PROBABLY 2148 counts per rotation
        //Wheels are 96mm diameter
        double countsPerMM = 2148 / (96 * Math.PI);

        flMotor.setTargetPosition(flMotor.getCurrentPosition() + (int) (distance * countsPerMM));
        brMotor.setTargetPosition(brMotor.getCurrentPosition() + (int) (distance * countsPerMM));
        frMotor.setPower(1);
        blMotor.setPower(1);

        while (flMotor.isBusy() || brMotor.isBusy()) {
            ;
        }

        frMotor.setPower(0);
        blMotor.setPower(0);
    }
}
