package org.firstinspires.ftc.teamcode.Examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "flywheel", group = "Examples")
public class flywheelExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor flywheel = hardwareMap.dcMotor.get("flywheelMotor");

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        double flywheelPower = 0;

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.back) {
                // Toggles the flywheel power between 0 and 1
                flywheelPower=1-flywheelPower;
                //if flywheelPower is greater than 0, make it 0
                //otherwise, make it 1
            }
            flywheel.setPower(flywheelPower);
            sleep(100); // to make sure one press is not registered as multiple
        }
    }
}
