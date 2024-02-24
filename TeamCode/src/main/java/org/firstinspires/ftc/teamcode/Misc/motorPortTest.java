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

        DcMotor slideMotorR = hardwareMap.dcMotor.get("motorSlideR");
        DcMotor slideMotorL = hardwareMap.dcMotor.get("motorSlideL");
        Servo armTopServoR = hardwareMap.servo.get("armTopServoR");
        Servo armTopServoL = hardwareMap.servo.get("armTopServoL");
        DcMotor hangMotor = hardwareMap.dcMotor.get("hangMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo droneServo = hardwareMap.servo.get("droneServo");

        //reverse right side motors. reverse left side if goes backwards
//        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addLine("Hold down the button to run each motor");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            String active = "active:\n";

            if (gamepad1.a){
                frMotor.setPower(1);
                active+="frMotor\n";
            } else {
                frMotor.setPower(0);
            }
            telemetry.addLine("A - Front Right Wheel: Control Hub Port 0");


            if (gamepad1.b){
                brMotor.setPower(1);
                active+="brMotor\n";
            } else {
                brMotor.setPower(0);
            }
            telemetry.addLine("B - Back Right Wheel: Control Hub Port 1");


            if (gamepad1.x){
                blMotor.setPower(1);
                active+="blMotor\n";
            } else {
                blMotor.setPower(0);
            }
            telemetry.addLine("X - Back Left Wheel: Control Hub Port 2");


            if (gamepad1.y){
                flMotor.setPower(1);
                active+="flMotor\n";
            } else {
                flMotor.setPower(0);
            }
            telemetry.addLine("Y - Back Left Wheel: Control Hub Port 3");


            if (gamepad1.dpad_up){
                slideMotorR.setPower(1);
                slideMotorL.setPower(1);
                active+="slideMotorR and slideMotorL\n";
            } else {
                slideMotorR.setPower(0);
                slideMotorL.setPower(0);

            }
            telemetry.addLine("D-Pad Up - Linear Slide R and L: Expansion Hub Port 0 and 2 with Encoder");


            if (gamepad1.dpad_down){
                hangMotor.setPower(1);
                active+="armRotateMotor\n";
            } else {
                hangMotor.setPower(0);
            }
            telemetry.addLine("D-Pad Down - Hang Motor: Expansion Hub Port 3 with Encoder?");


            if (gamepad1.dpad_left || gamepad1.dpad_right){
                intakeMotor.setPower(1);
                active+="intakeMotor\n";
            } else {
                intakeMotor.setPower(0);
            }
            telemetry.addLine("D-Pad Left or Right - Intake Motor: Expansion Hub Port 2");

            if (gamepad1.left_bumper){
                armTopServoR.setPosition(1-armTopServoR.getPosition());
                armTopServoL.setPosition(1-armTopServoL.getPosition());

            }
            telemetry.addLine("Left Bumper - Arm Top Servo R and L: Control Hub Port 0 and 1");


            if (gamepad1.right_bumper){
                droneServo.setPosition(1-droneServo.getPosition());
            }
            telemetry.addLine("Right Bumper - Drone Servo: Control Hub Port 1");
            telemetry.addData("Front Right Wheel Encoder", frMotor.getCurrentPosition());
            telemetry.addData("Front Left Wheel Encoder", flMotor.getCurrentPosition());
            telemetry.addData("Back Right Wheel Encoder", brMotor.getCurrentPosition());
            telemetry.addData("Back Left Wheel Encoder", blMotor.getCurrentPosition());
            telemetry.addData("Controller Status", gamepad1.toString());
            telemetry.addLine(active);
            telemetry.update();
        }
    }
}
