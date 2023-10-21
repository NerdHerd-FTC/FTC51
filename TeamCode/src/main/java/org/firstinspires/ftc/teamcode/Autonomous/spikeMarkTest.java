package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="spikemarktest", group = "Tests")
public class spikeMarkTest extends autoExample {
    private final DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
    private final DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
    private final DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
    private final DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");

    private final DcMotor slideMotor = hardwareMap.dcMotor.get("motorSlide");
    private final Servo armTopServo = hardwareMap.servo.get("armTopServo");
    private final DcMotor armRotateMotor = hardwareMap.dcMotor.get("armRotateMotor");
    private final DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
    private final Servo droneServo = hardwareMap.servo.get("droneServo");

    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.y){
                positionCenter();
            }
            if (gamepad1.x){
                positionLeft();
            }
            if (gamepad1.b){
                positionRight();
            }
            telemetry.update();
        }
    }

    public void positionCenter() {
        driveFunc(100);
    }

    public void positionRight(){
        strafe(100,1);
    }
    public void positionLeft(){
        strafe(100,-1);
    }
}
