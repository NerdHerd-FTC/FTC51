package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Red Away from Audience")
public class redAway extends autoExample {
    @Override
    public void runOpMode() throws InterruptedException {
//        String[] telemetries = new String[10];
//        for(int i = 0; i < telemetries.length; i++){
//            telemetries[i] = "";
//        }


        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");
        DcMotor slideMotor = hardwareMap.dcMotor.get("motorSlide");

        // fix drivetrain motor directions
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD); // IDK MAN, WE NEED TO TEST
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset the encoder
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frMotor.setTargetPosition(0);
        flMotor.setTargetPosition(0);
        brMotor.setTargetPosition(0);
        blMotor.setTargetPosition(0);
        slideMotor.setTargetPosition(0);

        frMotor.setPower(0.5);
        flMotor.setPower(0.5);
        brMotor.setPower(0.5);
        blMotor.setPower(0.5);
        slideMotor.setPower(0.5);

        // Sets motors into go to position mode
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Switches from setting power to moving to a position

        telemetry.addLine("Ready to Run");
        telemetry.addLine("Red Alliance, Away from Audience, Front facing backdrop");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Running");
        telemetry.update();
        //left is -1, right is 1
        strafe(200,-1,frMotor,brMotor,flMotor,blMotor);
        telemetry.addLine("Strafe Finished");
        telemetry.update();
        driveFunc(450,frMotor,brMotor,flMotor,blMotor);
        telemetry.addLine("OpMode Finished");
        telemetry.update();
    }
}