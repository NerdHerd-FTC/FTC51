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
        float chicken = 100F;
        while (opModeIsActive()) {
            number++;
            chicken *= 1.5F;
            chicken -= Math.random() * 100;
            chicken = chicken%100;
            telemetry.addData("the secret number", 10 - number);
            telemetry.addData("the secret number pt 2", number + 13);
            telemetry.addData("very important magic number...", chicken);
            telemetry.update();
            sleep(50);
        }
    }
}