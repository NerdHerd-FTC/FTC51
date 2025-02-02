package org.firstinspires.ftc.teamcode.Drivetrains;

//javadocs here: https://javadoc.io/doc/org.firstinspires.ftc
//ftc docs here: https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Drivetrains.armControls;

// Drive train controls for mecanum drive
// Mecanum drive allows omnidirectional movement

@TeleOp(name = "Mecanum - RO")
public class mecanumDriveRO_Arm extends LinearOpMode{

    final private armControls armControl = new armControls();
    final private mecanumDriveRO driveControl = new mecanumDriveRO();
    @Override
    public void runOpMode() throws InterruptedException {

        // Stick input is multiplied by these variables
        double strafe_speed=1;

//      boolean intakeButtonPressed = false;
//      boolean droneLaunched = false;

        //create objects for motors
        // f=front, b=back, l=left, r=right
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");


        DcMotor slideMotorR = hardwareMap.dcMotor.get("motorSlideR");
        DcMotor slideMotorL = hardwareMap.dcMotor.get("motorSlideL");
        Servo armTopServoR = hardwareMap.servo.get("armTopServoR");
        Servo armTopServoL = hardwareMap.servo.get("armTopServoL");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo droneServo = hardwareMap.servo.get("droneServo");
        Servo intakeServoR = hardwareMap.servo.get("intakeServoR");
        Servo intakeServoM = hardwareMap.servo.get("intakeServoM");
        Servo intakeServoL = hardwareMap.servo.get("intakeServoL");


        // fix drivetrain motor directions
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        armTopServoR.setDirection(Servo.Direction.REVERSE);
        armTopServoL.setDirection(Servo.Direction.FORWARD);
        intakeServoL.setDirection(Servo.Direction.REVERSE);
        intakeServoR.setDirection(Servo.Direction.FORWARD);
        intakeServoM.setDirection(Servo.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // tell motors to brake when not active
        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // reset encoder
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Turn off encoders for drivetrain
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        boolean slowMode = false;
        boolean slowModePressed = false;

        // Display controls
        telemetry.addLine("Mecanum Drive - Robot Oriented");
        telemetry.addLine();
        telemetry.addLine("Variables initialized");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("Gamepad 1");
        telemetry.addLine("Left Stick - Move (Forward, Backward, Strafe)");
        telemetry.addLine("Right Stick - Rotate");
        telemetry.addLine("A button - Toggle intake on/off");
        telemetry.addLine("B button - Toggle intake direction");
        telemetry.addLine("Back - Drone Release");
        telemetry.addLine("Y - Slow Forward Move");
        telemetry.addLine("Gamepad 2");
        telemetry.addLine("RT - Extend Arm");
        telemetry.addLine("LT - Retract Arm");
        telemetry.addLine("RB - Rotate Arm Servo to Release Position");
        telemetry.addLine("LB - Rotate Arm Servo to Intake Position");
        telemetry.addLine();
        telemetry.addLine("✅ Ready to start ✅");
        telemetry.addLine("redy to star, more like... you wount");
        telemetry.addLine("duenes yuo owe me a $dollar");
        telemetry.update();

        //waits for start of game
        waitForStart();

        droneServo.setPosition(1);
        armTopServoL.setPosition(org.firstinspires.ftc.teamcode.Drivetrains.armControls.servoLoadPosition);
        armTopServoR.setPosition(org.firstinspires.ftc.teamcode.Drivetrains.armControls.servoLoadPosition);
        intakeServoL.setPosition(0.5);
        intakeServoR.setPosition(0.5);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            // Toggle slow mode on and off
            if (gamepad1.y){
                if (!slowModePressed){
                    slowModePressed=true;
                    slowMode=!slowMode;
                }
            } else {slowModePressed=false;}

            driveControl.mecanumDrive(flMotor,frMotor,blMotor,brMotor,strafe_speed,(slowMode) ? .5 : 1,gamepad1);
            String armTelemetry = armControl.armControls(slideMotorR,slideMotorL,armTopServoR,armTopServoL,intakeMotor,droneServo,intakeServoL, intakeServoM, intakeServoR,gamepad1,gamepad2);

            telemetry.addLine(armTelemetry);
            telemetry.addData("Slowmode enabled",slowMode);
            telemetry.addData("Timer","%.2f", timer.time());
            telemetry.addData("Gamepad 1 Status:",gamepad1.toString());
            telemetry.addData("Gamepad 2 Status:",gamepad2.toString());
            telemetry.update();
        }
    }
}