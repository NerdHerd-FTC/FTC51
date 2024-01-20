package org.firstinspires.ftc.teamcode.Drivetrains;

//javadocs here: https://javadoc.io/doc/org.firstinspires.ftc
//ftc docs here: https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Drive train controls for mecanum drive
// Mecanum drive allows omnidirectional movement

@TeleOp(name = "Mecanum - RO")
public class mecanumDriveRO_Arm extends armControls{
    @Override
    public void runOpMode() throws InterruptedException {

        // Stick input is multiplied by these variables
        double strafe_speed=1;
        double turn_speed=0.75;

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
        DcMotor hangMotor = hardwareMap.dcMotor.get("hangMotor");

        // fix drivetrain motor directions
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        armTopServoR.setDirection(Servo.Direction.REVERSE);
        armTopServoL.setDirection(Servo.Direction.FORWARD);
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
//        frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        droneServo.setPosition(1);

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
        telemetry.addLine("Back - Drone Release");
        telemetry.addLine("Gamepad 2");
        telemetry.addLine("RT - Extend Arm");
        telemetry.addLine("LT - Retract Arm");
        telemetry.addLine("RB - Rotate Arm Servo to Release Position");
        telemetry.addLine("LB - Rotate Arm Servo to Intake Position");
        telemetry.addLine("D-Pad Up - Rotate Arm Body Up");
        telemetry.addLine("D-Pad Down - Rotate Arm Body Down");
        telemetry.addLine();
        telemetry.addLine("✅ Ready to start ✅");
        telemetry.addLine("redy to star, more like... you wount");
        telemetry.update();

        //waits for start of game
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            //gamepad variables
            double stickY = -gamepad1.left_stick_y; //Y stick value is REVERSED
            double stickX = gamepad1.left_stick_x;
            double rStickX = gamepad1.right_stick_x*turn_speed;


            // Calculate denominator
            double denominator = Math.max(Math.max(Math.abs(stickX*strafe_speed),Math.abs(stickY)),Math.max(Math.abs(rStickX), 1));
            // If any of the values are greater then one, then this value will divide them
            // Insures that every value will have the correct ratio
            // If stickX is one and rStick X is one, then without denominator flmotor will be set to 2
            // any value above one in the set power function works as same as just putting in one
            // leading to incorrect movement
            // denominator fixes this, by dividing everything by the sum.
            // (If less then one, there isn't any issue, so divide by one)

            // Moves all the motors forwards slowly
            // Helps with moving slowly to score
            if (gamepad1.dpad_up) {
                stickY += 0.1;
            }

            // Move motors
            flMotor.setPower((stickY + (stickX*strafe_speed) + rStickX) / denominator);
            frMotor.setPower((stickY - (stickX*strafe_speed) - rStickX) / denominator);
            blMotor.setPower((stickY - (stickX*strafe_speed) + rStickX) / denominator);
            brMotor.setPower((stickY + (stickX*strafe_speed) - rStickX) / denominator);
            // stickY moves forward, so positive effect on every motor
            // stickX is strafe, so positive for fl and br, and negative for fr and bl
            // rStickX is rotate, so positive for fl and bl, and negative for fr and br

            armControls(slideMotorR,slideMotorL,armTopServoR,armTopServoL,intakeMotor,droneServo,hangMotor);

            telemetry.addData("Timer","%.2f", timer.time());
            telemetry.addData("Gamepad 1 Status:",gamepad1.toString());
            telemetry.addData("Gamepad 2 Status:",gamepad2.toString());
            telemetry.update();
        }
    }
}