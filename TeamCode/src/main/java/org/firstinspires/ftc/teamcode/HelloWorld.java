package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "Hello World", group="Robot")
public class HelloWorld extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("Hello World!");
        telemetry.addLine("hello world......");
        telemetry.update();
        waitForStart();
        int number = 0;
        while (opModeIsActive()) {
            number++;
            telemetry.addData("the secret number", 10 - number);
            telemetry.addData("the secret number pt 2", number + 13);
            telemetry.update();
            sleep(100);
        }
    }
}