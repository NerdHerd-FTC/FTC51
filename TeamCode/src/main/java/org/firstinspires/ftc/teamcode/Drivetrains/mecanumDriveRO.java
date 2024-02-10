package org.firstinspires.ftc.teamcode.Drivetrains;

//javadocs here: https://javadoc.io/doc/org.firstinspires.ftc
//ftc docs here: https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

// Drive train controls for mecanum drive
// Mecanum drive allows omnidirectional movement

@TeleOp(name = "Mecanum - RO - NO ARM CONTROLS", group="Drive Tests")
public class mecanumDriveRO extends LinearOpMode {

    public void mecanumDrive(DcMotor flMotor, DcMotor frMotor, DcMotor blMotor, DcMotor brMotor, double strafe_speed, Gamepad gamepad1){
        // get values from controller
        double stickY = -gamepad1.left_stick_y; //Y stick value is REVERSED
        double stickX = gamepad1.left_stick_x;
        double rStickX = gamepad1.right_stick_x;

        // get denominator
        double denominator = Math.max(Math.abs(stickX) + Math.abs(stickY) + Math.abs(rStickX), 1);
        // denominator ensures ratios are maintained, because the motors only go from 0-1

        // set values based on mecanum drive
        flMotor.setPower((stickY + (stickX*strafe_speed) + rStickX) / denominator);
        frMotor.setPower((stickY - (stickX*strafe_speed) - rStickX) / denominator);
        blMotor.setPower((stickY - (stickX*strafe_speed) + rStickX) / denominator);
        brMotor.setPower((stickY + (stickX*strafe_speed) - rStickX) / denominator);
    }

    public void mecanumDrive(DcMotor flMotor, DcMotor frMotor, DcMotor blMotor, DcMotor brMotor, double strafe_speed, double forward_speed, Gamepad gamepad1){
        // get values from controller
        double stickY = -gamepad1.left_stick_y; //Y stick value is REVERSED
        double stickX = gamepad1.left_stick_x;
        double rStickX = gamepad1.right_stick_x;

        // get denominator
        double denominator = Math.max(Math.abs(stickX) + Math.abs(stickY) + Math.abs(rStickX), 1);
        // denominator ensures ratios are maintained, because the motors only go from 0-1

        // set values based on mecanum drive
        flMotor.setPower((stickY + (stickX*strafe_speed) + rStickX) / denominator);
        frMotor.setPower((stickY - (stickX*strafe_speed) - rStickX) / denominator);
        blMotor.setPower((stickY - (stickX*strafe_speed) + rStickX) / denominator);
        brMotor.setPower((stickY + (stickX*strafe_speed) - rStickX) / denominator);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        //create objects for motors
        // f=front, b=back, l=left, r=right
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");

        // Turn off encoders for drivetrain
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready to start");
        telemetry.update();

        //waits for start of game
        waitForStart();

        while (opModeIsActive()) {
            mecanumDrive(flMotor, frMotor, blMotor, brMotor, 1.0, gamepad1);
        }
    }
}
