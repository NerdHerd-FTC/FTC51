package org.firstinspires.ftc.teamcode.Misc;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Motor Port Test", group = "Tests")
public class motorPortTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        //get all motors attached to robot
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");

        DcMotor slideMotor = hardwareMap.dcMotor.get("motorSlide");
        Servo armTopServo = hardwareMap.servo.get("armTopServo");
        DcMotor armRotateMotor = hardwareMap.dcMotor.get("armRotateMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo droneServo = hardwareMap.servo.get("droneServo");

        //reverse right side motors. reverse left side if goes backwards
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armTopServo.setDirection(Servo.Direction.FORWARD);
        armRotateMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Hold down the button to run each motor");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.a){
                frMotor.setPower(1);
            } else {
                frMotor.setPower(0);
            }
            telemetry.addLine("A - Front Right Wheel: Control Hub Port 0");


            if (gamepad1.b){
                brMotor.setPower(1);
            } else {
                brMotor.setPower(0);
            }
            telemetry.addLine("B - Back Right Wheel: Control Hub Port 1 with Encoder");


            if (gamepad1.x){
                blMotor.setPower(1);
            } else {
                blMotor.setPower(0);
            }
            telemetry.addLine("X - Back Left Wheel: Control Hub Port 2");


            if (gamepad1.y){
                flMotor.setPower(1);
            } else {
                flMotor.setPower(0);
            }
            telemetry.addLine("Y - Back Left Wheel: Control Hub Port 3 with Encoder");


            if (gamepad1.dpad_up){
                slideMotor.setPower(1);
            } else {
                slideMotor.setPower(0);
            }
            telemetry.addLine("D-Pad Up - Linear Slide: Expansion Hub Port 0 with Encoder");


            if (gamepad1.dpad_down){
                armRotateMotor.setPower(1);
            } else {
                armRotateMotor.setPower(0);
            }
            telemetry.addLine("D-Pad Down - Arm Rotation Motor: Expansion Hub Port 1 with Encoder");


            if (gamepad1.dpad_left || gamepad1.dpad_right){
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(0);
            }
            telemetry.addLine("D-Pad Left or Right - Intake Motor: Expansion Hub Port 2");

            if (gamepad1.left_bumper){
                armTopServo.setPosition(1-armTopServo.getPosition());
            }
            telemetry.addLine("Left Bumper - Arm Top Servo: Control Hub Port 0");


            if (gamepad1.right_bumper){
                droneServo.setPosition(1-droneServo.getPosition());
            }
            telemetry.addLine("Right Bumper - Drone Servo: Control Hub Port 1");
            telemetry.addData("Front Right Wheel Encoder", frMotor.getCurrentPosition());
            telemetry.addData("Front Left Wheel Encoder", flMotor.getCurrentPosition());
            telemetry.addData("Back Right Wheel Encoder", brMotor.getCurrentPosition());
            telemetry.addData("Back Left Wheel Encoder", blMotor.getCurrentPosition());
            telemetry.addData("Controller Status", gamepad1.toString());
            telemetry.update();
        }
    }
}
