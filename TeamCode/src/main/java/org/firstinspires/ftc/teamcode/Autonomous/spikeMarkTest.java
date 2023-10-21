package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="spikemarktest", group = "Tests")
public class spikeMarkTest extends autoExample {
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
