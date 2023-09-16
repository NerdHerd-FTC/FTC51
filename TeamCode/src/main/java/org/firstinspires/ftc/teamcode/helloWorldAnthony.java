package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Hello World", group="Robot")
public class helloWorldAnthony extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Hello World (wow)");
        telemetry.update();

        waitForStart();

        int timeRan = 0;

        while (opModeIsActive()) {
            timeRan++;
            telemetry.addData("nanoseconds", timeRan*1000000000);
            telemetry.addData("Seconds Ran",timeRan);
            telemetry.addData("real number for reals", Math.random());
            telemetry.addData("Seconds but double", timeRan*2);

            telemetry.update();

            sleep(100);
        }
    }
}
