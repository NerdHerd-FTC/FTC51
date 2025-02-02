package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Auto Example", group = "Examples")
@Disabled
public class autoExample extends LinearOpMode {

    // Immutable

    @Override
    public void runOpMode() throws InterruptedException {
        String[] telemetries = new String[10];
        for(int i = 0; i < telemetries.length; i++){
            telemetries[i] = "";
        }

        // Define motor variables
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");
        DcMotor slideMotor = hardwareMap.dcMotor.get("motorSlide");

        Servo armTopServo = hardwareMap.servo.get("armTopServo");
        DcMotor armRotateMotor = hardwareMap.dcMotor.get("armRotateMotor"); // baned 😮‍💨
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo droneServo = hardwareMap.servo.get("droneServo");

        telemetries=print("Started",telemetries);

        // Set motor directions
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetries=print("directions set",telemetries);

        // Reset the encoder
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetries=print("mode set",telemetries);


        flMotor.setTargetPosition(0);
        brMotor.setTargetPosition(0);
        frMotor.setTargetPosition(0);
        blMotor.setTargetPosition(0);
        slideMotor.setTargetPosition(0);
        telemetries=print("positions set",telemetries);

        flMotor.setPower(1);
        brMotor.setPower(1);
        frMotor.setPower(1);
        blMotor.setPower(1);
        slideMotor.setPower(1);
        telemetries=print("power set",telemetries);

        // Sets motors into go to position mode
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Switches from setting power to moving to a position
        telemetries=print("modes set",telemetries);

        waitForStart();

        // Move robot forward 150 millimeters
        telemetries=print("starting drive",telemetries);
        driveFunc(150,frMotor,brMotor,flMotor,blMotor);
        print("drive has finished",telemetries);

        //strafe(100,1);
    }

    //First part is "Encoder Resolution Formula" from goBilda
    //    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    final double countsPerMM = ((((1+(46/17))) * (1+(46/11))) * 28) / (96 * Math.PI);
    // Used to calculate the amount to move each motor

    public void driveFunc(double distance, DcMotor frMotor, DcMotor brMotor, DcMotor flMotor, DcMotor blMotor) throws InterruptedException {
        // Moves the robot the distance forward
        // Distance is in millimeters

        flMotor.setTargetPosition(flMotor.getCurrentPosition() + (int) (distance * countsPerMM)); // Tell motors to move
        brMotor.setTargetPosition(brMotor.getCurrentPosition() + (int) (distance * countsPerMM));
        frMotor.setTargetPosition(frMotor.getCurrentPosition() + (int) (distance * countsPerMM));
        blMotor.setTargetPosition(blMotor.getCurrentPosition() + (int) (distance * countsPerMM));

        while (flMotor.isBusy() || brMotor.isBusy() || frMotor.isBusy() || blMotor.isBusy()){Thread.sleep(1000);}//wait
    }//

    public void strafe(double distance, double direction, DcMotor frMotor, DcMotor brMotor, DcMotor flMotor, DcMotor blMotor) throws InterruptedException {

        flMotor.setTargetPosition(flMotor.getCurrentPosition() + (int) ((distance * countsPerMM) * direction));
        brMotor.setTargetPosition(brMotor.getCurrentPosition() + (int) ((distance * countsPerMM) * direction));
        frMotor.setTargetPosition(frMotor.getCurrentPosition() - (int) ((distance * countsPerMM) * direction));
        blMotor.setTargetPosition(blMotor.getCurrentPosition() - (int) ((distance * countsPerMM) * direction));

        while (flMotor.isBusy() || brMotor.isBusy() || frMotor.isBusy() || blMotor.isBusy()){Thread.sleep(1000);}//wait
    }

    public void rotate(double distance, DcMotor frMotor, DcMotor brMotor, DcMotor flMotor, DcMotor blMotor) throws InterruptedException {
        // Moves the robot the distance forward
        // Distance is in millimeters clockwise

        flMotor.setTargetPosition(flMotor.getCurrentPosition() + (int) (distance * countsPerMM)); // Tell motors to move
        brMotor.setTargetPosition(brMotor.getCurrentPosition() - (int) (distance * countsPerMM));
        frMotor.setTargetPosition(frMotor.getCurrentPosition() - (int) (distance * countsPerMM));
        blMotor.setTargetPosition(blMotor.getCurrentPosition() + (int) (distance * countsPerMM));

        while (flMotor.isBusy() || brMotor.isBusy() || frMotor.isBusy() || blMotor.isBusy()){Thread.sleep(1000);}//wait
    }

//    public String[] telemetries = new String[10];

    public String[] print(String text,String[] telemetries){
        // shift all elements
        for(int i = telemetries.length - 2; i >= 0; i--){
            telemetries[i + 1] = telemetries[i];
        }
        telemetries[0]=text;

        for(int i = 0;i<telemetries.length;i++){
            telemetry.addLine(telemetries[i]);
        }
        telemetry.update();

        return telemetries;
    }

}