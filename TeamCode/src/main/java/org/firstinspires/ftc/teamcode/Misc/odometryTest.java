package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name= "odometry test", group = "Tests")
public class odometryTest extends LinearOpMode{
    // https://youtu.be/Av9ZMjS--gY?si=L-hWFuGXNby_seRZ
    // https://youtu.be/lpVG2Pl6RGY?t=262
    @Override
    public void runOpMode(){
        //create objects for motors
        // f=front, b=back, l=left, r=right
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");

        DcMotor leftEncoder = brMotor;
        DcMotor rightEncoder = blMotor;
        DcMotor frontEncoder = frMotor;

        // fix motor directions
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Turn off encoders for drivetrain
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("ready to start");
        telemetry.addData("Starting position left odometry pod",leftEncoder.getCurrentPosition());
        telemetry.addData("Starting position right odometry pod",rightEncoder.getCurrentPosition());
        telemetry.addData("Starting position front odometry pod",frontEncoder.getCurrentPosition());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //gamepad variables
            double stickY = -gamepad1.left_stick_y; //Y stick value is REVERSED
            double stickX = gamepad1.left_stick_x;
            double rStickX = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(stickX) + Math.abs(stickY) + Math.abs(rStickX), 1);

            flMotor.setPower((stickY + stickX + rStickX) / denominator);
            frMotor.setPower((stickY - stickX - rStickX) / denominator);
            blMotor.setPower((stickY - stickX + rStickX) / denominator);
            brMotor.setPower((stickY + stickX - rStickX) / denominator);

            telemetry.addData("Current position left odometry pod",leftEncoder.getCurrentPosition());
            telemetry.addData("Current position right odometry pod",rightEncoder.getCurrentPosition());
            telemetry.addData("Current position front odometry pod",frontEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
