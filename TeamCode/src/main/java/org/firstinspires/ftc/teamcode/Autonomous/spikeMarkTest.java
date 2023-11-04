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
//        String[] telemetries = new String[10];
//        for(int i = 0; i < telemetries.length; i++){
//            telemetries[i] = "";
//        }


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
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset the encoder
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // fix drivetrain motor directions
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD); // IDK MAN, WE NEED TO TEST
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        frMotor.setTargetPosition(0);
        flMotor.setTargetPosition(0);
        brMotor.setTargetPosition(0);
        blMotor.setTargetPosition(0);
        slideMotor.setTargetPosition(0);
        armRotateMotor.setTargetPosition(0);

        frMotor.setPower(1); // IDK MAN, WE NEED TO TEST
        flMotor.setPower(1);
        brMotor.setPower(1);
        blMotor.setPower(1);
        slideMotor.setPower(1);
        armRotateMotor.setPower(1);

        // Sets motors into go to position mode
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Switches from setting power to moving to a position

        waitForStart();

        armTopServo.setPosition(0.3);//get arm servo off of intake

        while(opModeIsActive()){
            if (gamepad1.y){
                positionCenter(frMotor,brMotor,flMotor,blMotor, slideMotor, armRotateMotor, armTopServo);
            }
            if (gamepad1.x){
                positionLeft(frMotor,brMotor,flMotor,blMotor, slideMotor, armRotateMotor, armTopServo);
            }
            if (gamepad1.b){
                positionRight(frMotor,brMotor,flMotor,blMotor, slideMotor, armRotateMotor, armTopServo);
            }
            telemetry.addData("flMotor encoder", flMotor.getCurrentPosition());
            telemetry.addData("frMotor encoder", frMotor.getCurrentPosition());
            telemetry.addData("blMotor encoder", blMotor.getCurrentPosition());
            telemetry.addData("brMotor encoder", brMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    public void positionCenter(DcMotor frMotor, DcMotor brMotor, DcMotor flMotor, DcMotor blMotor, DcMotor slideMotor, DcMotor armRotateMotor, Servo armTopServo) throws InterruptedException {
        driveFunc(500,frMotor,brMotor,flMotor,blMotor);
//        slideMotor.setTargetPosition(600);
//        armRotateMotor.setTargetPosition(-200);
//        while(slideMotor.isBusy()){Thread.sleep(1000);}
//        armTopServo.setPosition(1);
    }

    public void positionRight(DcMotor frMotor, DcMotor brMotor, DcMotor flMotor, DcMotor blMotor, DcMotor slideMotor, DcMotor armRotateMotor, Servo armTopServo) throws InterruptedException {
        strafe(100,1,frMotor,brMotor,flMotor,blMotor);
    }
    public void positionLeft(DcMotor frMotor, DcMotor brMotor, DcMotor flMotor, DcMotor blMotor, DcMotor slideMotor, DcMotor armRotateMotor, Servo armTopServo) throws InterruptedException {
        strafe(100,-1,frMotor,brMotor,flMotor,blMotor);
    }
}
