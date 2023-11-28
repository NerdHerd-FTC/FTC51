package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="movement test")
public class movementTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");
        DcMotor slideMotor = hardwareMap.dcMotor.get("motorSlide");

        // fix drivetrain motor directions
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD); // IDK MAN, WE NEED TO TEST
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        move(100,300,frMotor,brMotor,flMotor,blMotor);
        move(100,120,frMotor,brMotor,flMotor,blMotor);
    }

    final double countsPerMM = 1000 / (96 * Math.PI);

    public double sin(double y){
        return Math.sin(Math.toRadians(y));
    }

    public void move(int distance, double angle, DcMotor frMotor, DcMotor brMotor, DcMotor flMotor, DcMotor blMotor) throws InterruptedException {
        double vr = sin(angle+45);
        double vl = sin(angle-45);
        int denominator = (int) Math.max(Math.abs(vr+vl),1);

        frMotor.setTargetPosition((int) ((vr/denominator)*distance*countsPerMM));
        blMotor.setTargetPosition((int) ((vr/denominator)*distance*countsPerMM));

        brMotor.setTargetPosition((int) ((vl/denominator)*distance*countsPerMM));
        flMotor.setTargetPosition((int) ((vl/denominator)*distance*countsPerMM));

        while (flMotor.isBusy() || brMotor.isBusy() || frMotor.isBusy() || blMotor.isBusy()){Thread.sleep(500);}//wait
    }
}
