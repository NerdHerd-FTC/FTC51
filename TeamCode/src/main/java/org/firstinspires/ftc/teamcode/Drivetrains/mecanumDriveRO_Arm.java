package org.firstinspires.ftc.teamcode.Drivetrains;

//javadocs here: https://javadoc.io/doc/org.firstinspires.ftc
//ftc docs here: https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// Drive train controls for mecanum drive
// Mecanum drive allows omnidirectional movement

@TeleOp(name = "Mecanum - RO")
public class mecanumDriveRO_Arm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //create objects for motors
        // f=front, b=back, l=left, r=right
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");

        DcMotor slideMotor = hardwareMap.dcMotor.get("motorSlide");
        Servo armTopServo = hardwareMap.servo.get("armTopServo");
        DcMotor armRotateMotor = hardwareMap.dcMotor.get("armRotateMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo droneServo = hardwareMap.servo.get("droneServo");

        // fix drivetrain motor directions
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD); // IDK MAN, WE NEED TO TEST
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armTopServo.setDirection(Servo.Direction.FORWARD);
        armRotateMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // tell motors to brake when not active
        armRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reset encoder
        armRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // configure motors to correct positions
        armRotateMotor.setPower(1);

        armRotateMotor.setTargetPosition(0);

        armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // encoders

        droneServo.setPosition(0);

        // initialize variables
        boolean droneLaunched = false;
        boolean intakeButtonPressed = false;

        double strafe_speed=0.75;

        // Display controls
        telemetry.addLine("Mecanum Drive - Robot Oriented");
        telemetry.addLine();
        telemetry.addLine("Variables initialized");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("Left Stick - Move (up, down, left, right)");
        telemetry.addLine("Right Stick- Turn");
        telemetry.addLine("Start - Reset Yaw");
        telemetry.addLine("RT - Extend Arm");
        telemetry.addLine("LT - Retract Arm");
        telemetry.addLine("RB - Rotate Arm Servo to Forward Position (150 Deg)");
        telemetry.addLine("LB - Rotate Arm Servo to 0 Degrees");
        telemetry.addLine("D-Pad Up - Rotate Arm Body Forwards");
        telemetry.addLine("D-Pad Down - Rotate Arm Body Backwards");
        telemetry.addLine("A button - Toggle intake on/off");
        telemetry.addLine();
        telemetry.addLine("Ready to start");
        telemetry.update();

        //waits for start of game
        waitForStart();

        while (opModeIsActive()) {
            //gamepad variables
            double stickY = -gamepad1.left_stick_y; //Y stick value is REVERSED
            double stickX = gamepad1.left_stick_x;
            double rStickX = gamepad1.right_stick_x;

            //triggers for arm extending
            double rTrigger = gamepad1.right_trigger;
            double lTrigger = gamepad1.left_trigger;

            //in our case, our servos have a range of 300 degrees
            //the position numbers are a fraction of these 300 degrees
            // The position goes from 0 to 1
            // This represents the fraction of 300 degrees the motor should be at
            // eg. 0.5 would be 150 degrees & 0.1 would be 30.
            if (gamepad1.right_bumper){
                armTopServo.setPosition(0.6);
            } else if (gamepad1.left_bumper) {
                armTopServo.setPosition(0.125);
            }

            if (gamepad1.a) { // Toggle intake motor on/off
                if (!intakeButtonPressed) {
                    intakeMotor.setPower(1 - intakeMotor.getPower());
                    telemetry.addLine("Intake toggled");
                    intakeButtonPressed = true;
                }
            } else {
                intakeButtonPressed = false;
            }


            // controls to rotate the whole arm up and down (forwards and backwards)
            // only changes position when the motor isn't busy, (hopefully) making controls more precise
            // 5700.4 counts per revolution
            if (gamepad1.dpad_up && !armRotateMotor.isBusy()) {
                armRotateMotor.setTargetPosition(armRotateMotor.getCurrentPosition() + 100); //makes the arm motors rotate forwards slowly
            } else if (gamepad1.dpad_down && !armRotateMotor.isBusy()) {
                armRotateMotor.setTargetPosition(armRotateMotor.getCurrentPosition() - 100); //makes the arm motors rotate backwards slowly
            }

            telemetry.addData("armPosition",armRotateMotor.getCurrentPosition());

            //moves the drone servo to the launch position
            if (gamepad1.back) {
                droneServo.setPosition(1);
            }
            telemetry.addData("Drone Launched",droneLaunched);

            double denominator = Math.max(Math.abs(stickX*strafe_speed) + Math.abs(stickY) + Math.abs(rStickX), 1);

            flMotor.setPower((stickY + (stickX*strafe_speed) + rStickX) / denominator);
            frMotor.setPower((stickY - (stickX*strafe_speed) - rStickX) / denominator);
            blMotor.setPower((stickY - (stickX*strafe_speed) + rStickX) / denominator);
            brMotor.setPower((stickY + (stickX*strafe_speed) - rStickX) / denominator);

            slideMotor.setPower(rTrigger-lTrigger+0.05); // move slide motor
            // the 0.05 is to counteract gravity
            // telemetry.addData("current arm motion:",rTrigger-lTrigger);
            telemetry.addLine("im wheler and i love duenes");

            telemetry.update();
        }
    }
}