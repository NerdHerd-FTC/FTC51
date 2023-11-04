package org.firstinspires.ftc.teamcode.Drivetrains;

//javadocs here: https://javadoc.io/doc/org.firstinspires.ftc
//ftc docs here: https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Drive train controls for mecanum drive
// Mecanum drive allows omnidirectional movement

@TeleOp(name = "Mecanum - RO - NO ARM CONTROLS", group="Drive Tests")
public class mecanumDriveRO extends LinearOpMode {
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
            //gamepad variables
            double stickY = -gamepad1.left_stick_y; //Y stick value is REVERSED
            double stickX = gamepad1.left_stick_x;
            double rStickX = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(stickX) + Math.abs(stickY) + Math.abs(rStickX), 1);

            flMotor.setPower((stickY + stickX + rStickX) / denominator);
            frMotor.setPower((stickY - stickX - rStickX) / denominator);
            blMotor.setPower((stickY - stickX + rStickX) / denominator);
            brMotor.setPower((stickY + stickX - rStickX) / denominator);

//            telemetry.update();
        }
    }
}
