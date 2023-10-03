package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Rubber Band Example")
public class rubberExample extends LinearOpMode {
    private final Servo droneServo = hardwareMap.servo.get("droneServo");
    @Override
    public void runOpMode() throws InterruptedException{
        //change the position to whatever the starting position should be
        droneServo.setPosition(0);

        telemetry.addLine("Ready for Launch");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.back){
                droneServo.setPosition(1);
                telemetry.addLine("Drone Launched");
                telemetry.update();
            }
        }

    }
}
