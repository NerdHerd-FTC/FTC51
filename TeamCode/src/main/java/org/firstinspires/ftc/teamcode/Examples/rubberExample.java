package org.firstinspires.ftc.teamcode.Examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Rubber Band Example", group = "Examples")
public class rubberExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Servo droneServo = hardwareMap.servo.get("droneServo");

        //change the position to whatever the starting position should be
        droneServo.setPosition(0);

        telemetry.addLine("Ready for Launch");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Press back to launch drone");
        telemetry.update();

        while (opModeIsActive()){
            if (gamepad1.back){
                droneServo.setPosition(1);
                telemetry.addLine("Drone Launched");
                telemetry.update();
            }
        }

    }
}
