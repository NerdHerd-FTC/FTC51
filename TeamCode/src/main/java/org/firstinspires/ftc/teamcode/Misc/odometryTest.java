package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name= "odometry test", group = "Tests")
public class odometryTest extends LinearOpMode{
    // https://youtu.be/Av9ZMjS--gY?si=L-hWFuGXNby_seRZ
    // https://youtu.be/lpVG2Pl6RGY?t=262
    @Override
    public void runOpMode(){
        //create objects for motors
        // f=front, b=back, l=left, r=right
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");

        // fix motor directions
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Odometry pods will plug in to the motor encoder ports

        // If there is a motor in the port where the pod is connected, it needs to be set to run
        // without encoder mode

        telemetry.addLine("ready to start");
        telemetry.update();

        waitForStart();
    }
}
