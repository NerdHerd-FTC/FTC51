package org.firstinspires.ftc.teamcode;

//javadocs here: https://ftctechnh.github.io/ftc_app/doc/javadoc/
//ftc docs here: https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Mecanum - DO")
public class mecanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //create objects for motors
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");

        //reverse right side motors. reverse left side if goes backwards
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //gets the IMU (Inertial Measurement Unit) from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");

        //sets orientation. change to match final robot
        //default is Logo Up and USB Forward
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        //waits for start of game
        waitForStart();

        while (opModeIsActive()) {
            //gamepad variables
            double stickY = -gamepad1.left_stick_y; //Y stick value is REVERSED
            double stickX = gamepad1.left_stick_x;
            double rStickX = gamepad1.right_stick_x;

            //this button should be hard to hit on accident
            //change if necessary
            if (gamepad1.x) {
                imu.resetYaw();
            }

            //retrieves the yaw of the robot
            double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            //calculates how much the robot should turn
            //directions are absolute
            //ex. if the stick is moved up, the robot will turn to move away from
            //the drive team, rather than moving forward
            double rotationX = stickX * Math.cos(-robotYaw) - stickY * Math.sin(-robotYaw);
            double rotationY = stickX * Math.sin(-robotYaw) + stickY * Math.cos(-robotYaw);

            rotationX = rotationX * 1.1; //counteract imperfect strafing

            //denominator is either the motor power or 1, depending on which is larger.
            //ensures all powers are the same ratio, but only when
            //at least one power is <-1 or >1
            double denominator = Math.max(Math.abs(rotationY) + Math.abs(rotationX) + Math.abs(rStickX), 1);

            double flPower = (rotationY + rotationX + rStickX) / denominator;
            double frPower = (rotationY - rotationX - rStickX) / denominator;
            double blPower = (rotationY - rotationX + rStickX) / denominator;
            double brPower = (rotationY + rotationX - rStickX) / denominator;

            flMotor.setPower(flPower);
            frMotor.setPower(frPower);
            blMotor.setPower(blPower);
            brMotor.setPower(brPower);

        }
    }
}
