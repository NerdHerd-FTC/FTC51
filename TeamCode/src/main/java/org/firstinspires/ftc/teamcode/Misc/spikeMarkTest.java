package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="spikemarktest", group = "Tests")
public class spikeMarkTest extends LinearOpMode {
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
    }

    public void positionCenter() {

    }
}
