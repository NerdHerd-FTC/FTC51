package org.firstinspires.ftc.teamcode.Drivetrains;

//javadocs here: https://javadoc.io/doc/org.firstinspires.ftc
//ftc docs here: https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Drive train controls for mecanum drive
// Mecanum drive allows omnidirectional movement

@TeleOp(name = "arcadeDrive", group = "Drive Tests")
@Disabled
public class arcadeDriveV1 extends LinearOpMode {
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
            // get stick values
            double lStickY = -gamepad1.left_stick_y; //Y stick value is REVERSED
            double rStickX = gamepad1.right_stick_x;

            if (Math.abs(lStickY) < Math.abs(rStickX)) {
                telemetry.addData("Turning",(rStickX>=0) ? "right":"left");
                flMotor.setPower(rStickX);
                blMotor.setPower(rStickX);

                frMotor.setPower(-rStickX);
                brMotor.setPower(-rStickX);

            } else {
                telemetry.addData("Moving",(rStickX>=0) ? "forward":"backward");
                flMotor.setPower(lStickY);
                blMotor.setPower(lStickY);

                frMotor.setPower(lStickY);
                brMotor.setPower(lStickY);
            }
            telemetry.addData("Left stick Y",lStickY);
            telemetry.addData("Right stick X",rStickX);
            telemetry.update();
        }
    }
}
