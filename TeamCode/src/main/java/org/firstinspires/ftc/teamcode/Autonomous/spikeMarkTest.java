package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="spikemarktest", group = "Tests")
public class spikeMarkTest extends autoExample {
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");
        DcMotor slideMotor = hardwareMap.dcMotor.get("motorSlide");

        Servo armTopServo = hardwareMap.servo.get("armTopServo");
        DcMotor armRotateMotor = hardwareMap.dcMotor.get("armRotateMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo droneServo = hardwareMap.servo.get("droneServo");

        // Set motor directions
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset the encoder
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // fix drivetrain motor directions
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE); // IDK MAN, WE NEED TO TEST
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flMotor.setTargetPosition(0);
        brMotor.setTargetPosition(0);
        slideMotor.setTargetPosition(0);

        flMotor.setPower(1);
        brMotor.setPower(1);
        slideMotor.setPower(1);

        // Sets motors into go to position mode
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Switches from setting power to moving to a position
        
        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.y){
                positionCenter(frMotor,brMotor,flMotor,blMotor);
            }
            if (gamepad1.x){
                positionLeft(frMotor,brMotor,flMotor,blMotor);
            }
            if (gamepad1.b){
                positionRight(frMotor,brMotor,flMotor,blMotor);
            }
            telemetry.update();
        }
    }

    public void positionCenter(DcMotor frMotor, DcMotor brMotor, DcMotor flMotor, DcMotor blMotor) {
        driveFunc(100,frMotor,brMotor,flMotor,blMotor);
    }

    public void positionRight(DcMotor frMotor, DcMotor brMotor, DcMotor flMotor, DcMotor blMotor){
        strafe(100,1,frMotor,brMotor,flMotor,blMotor);
    }
    public void positionLeft(DcMotor frMotor, DcMotor brMotor, DcMotor flMotor, DcMotor blMotor){
        strafe(100,-1,frMotor,brMotor,flMotor,blMotor);
    }
}
