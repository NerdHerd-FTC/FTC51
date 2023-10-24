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
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // configure motors to correct positions
        armRotateMotor.setPower(1);

        armRotateMotor.setTargetPosition(0);

        armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // encoders
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        droneServo.setPosition(0);

        // initialize variables
        boolean droneLaunched = false;
        boolean intakeButtonPressed = false;

        double strafe_speed=0.75;

        double gravityOffset=0.001;

        // 0.063 * 1/300
        double rotationFactor=0.063/300;

        // Display controls
        telemetry.addLine("Mecanum Drive - Robot Oriented");
        telemetry.addLine();
        telemetry.addLine("Variables initialized");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("Left Stick - Move (Forward, Backward, Strafe)");
        telemetry.addLine("Right Stick - Rotate");
        telemetry.addLine("RT - Extend Arm");
        telemetry.addLine("LT - Retract Arm");
        telemetry.addLine("RB - Rotate Arm Servo to Release Position");
        telemetry.addLine("LB - Rotate Arm Servo to Intake Position");
        telemetry.addLine("D-Pad Up - Rotate Arm Body Up");
        telemetry.addLine("D-Pad Down - Rotate Arm Body Down");
        telemetry.addLine("A button - Toggle intake on/off");
        telemetry.addLine();
        telemetry.addLine("✅ Ready to start ✅");
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

            if (gamepad1.a) { // Toggle intake motor on/off
                if (!intakeButtonPressed) {
                    intakeMotor.setPower(1 - intakeMotor.getPower());
                    intakeButtonPressed = true;
                }
            } else {
                intakeButtonPressed = false;
            }
            telemetry.addData("Intake power",intakeMotor.getPower());

            // controls to rotate the whole arm up and down (forwards and backwards)
            // only changes position when the motor isn't busy, (hopefully) making controls more precise
            // 5700.4 counts per revolution
            // 6.3 degrees per button press
            if (gamepad1.dpad_up && !armRotateMotor.isBusy()) {
                armRotateMotor.setTargetPosition(armRotateMotor.getCurrentPosition() + 100); //makes the arm motors rotate forwards slowly
            } else if (gamepad1.dpad_down && !armRotateMotor.isBusy()) {
                armRotateMotor.setTargetPosition(armRotateMotor.getCurrentPosition() - 100); //makes the arm motors rotate backwards slowly
            }

            // Servos have a range of 300 degrees
            double armServoPosition = 0.75 + (armRotateMotor.getTargetPosition()*rotationFactor);
            // Calculate what position to rotate arm to
            // 0.75 is the base
            // Then add the current arm position, times the rotation factor
            // rotation factor = 0.063*0.0033333...
            // 0.063 is about how much a single degree is in relation to the encoder output
            // 0.003333... is about how much a single degree is in relation to the servo's range
            if (gamepad1.right_bumper){
                armTopServo.setPosition(armServoPosition);
            } else if (gamepad1.left_bumper) {
                armTopServo.setPosition(0.125);
            }

            telemetry.addData("armPosition",armRotateMotor.getCurrentPosition());


            //moves the drone servo to the launch position
            if (gamepad1.back) {
                droneServo.setPosition(1);
                droneLaunched=true;
            }
            telemetry.addData("Drone Launched",droneLaunched);
            // Calculate denominator
            double denominator = Math.max(Math.abs(stickX*strafe_speed) + Math.abs(stickY) + Math.abs(rStickX), 1);
            // If any of the values are greater then one, then this value will divide them
            // Insures that every value will have the correct ratio
            // If stickX is one and rStick X is one, then without denominator flmotor will be set to 2
            // any value above one in the set power function works as same as just putting in one
            // leading to incorrect movement
            // denominator fixes this, by dividing everything by the sum.
            // (If less then one, there isn't any issue, so divide by one)

            // Move motors
            flMotor.setPower((stickY + (stickX*strafe_speed) + rStickX) / denominator);
            frMotor.setPower((stickY - (stickX*strafe_speed) - rStickX) / denominator);
            blMotor.setPower((stickY - (stickX*strafe_speed) + rStickX) / denominator);
            brMotor.setPower((stickY + (stickX*strafe_speed) - rStickX) / denominator);
            // stickY moves forward, so positive effect on every motor
            // stickX is strafe, so positive for fl and br, and negative for fr and bl
            // rStickX is rotate, so positive for fl and bl, and negative for fr and br


            if (slideMotor.getCurrentPosition()+rTrigger-lTrigger<1400){ // detect if upwards movement will go over
                slideMotor.setPower(rTrigger-lTrigger+gravityOffset); // move slide motor
            } else{
                slideMotor.setPower(-lTrigger+gravityOffset); // move slide motor only down
            }
            telemetry.addData("Slide position",slideMotor.getCurrentPosition());

            telemetry.addData("Gamepad Status:",gamepad1.toString());
            telemetry.update();
        }
    }
}