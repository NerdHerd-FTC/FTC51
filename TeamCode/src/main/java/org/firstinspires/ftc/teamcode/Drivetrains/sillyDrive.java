package org.firstinspires.ftc.teamcode.Drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="silly drive", group = "awesome")
public class sillyDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //create objects for motors
        // f=front, b=back, l=left, r=right
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");

        telemetry.addLine("Ready to start");
        telemetry.update();

        //waits for start of game
        waitForStart();

        while (opModeIsActive()) {
            double lStickY = -gamepad1.left_stick_y; //Y stick value is REVERSED
            double rStickY = -gamepad1.right_stick_y;

            double rTrigger = gamepad1.right_trigger;
            int rBumper = gamepad1.right_bumper ? -1 : 1;
            double lTrigger = gamepad1.left_trigger;
            int lBumper = gamepad1.left_bumper ? -1 : 1;

            flMotor.setPower(lTrigger*lBumper);
            frMotor.setPower(rTrigger*rBumper);
            blMotor.setPower(lStickY);
            brMotor.setPower(rStickY);
        }
    }
}
