package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class AutoMecanum extends LinearOpMode {

    RevBlinkinLedDriver lights;
    private CRServo servoIntake;

    boolean pGA2Y = false;
    boolean pGA2A = false;

    boolean pGA2B = false;
    boolean pGA2X = false;

    private PIDController controller;
public static double p = 0.0086, i = 0.9, d = 0.00023;
public static double f = 0.073;

    static int Offset = 700;

    public int ArmTarget = 0;
    private int stack = 490 - Offset;
    private int stack_height = 5;


    private boolean aButtonWasPressed = false;
    private boolean bButtonWasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

       DcMotor motorLeftLift = hardwareMap.get(DcMotorEx.class, "motorLeftLift");
       DcMotor motorRightLift = hardwareMap.get(DcMotorEx.class,"motorRightLift");

        motorLeftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        servoIntake = hardwareMap.get(CRServo.class,"servoIntake");

        motorLeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (d
        // .
        // river presses PLAY)

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double totalPower = (Math.abs(frontLeftPower) + Math.abs(backLeftPower)+Math.abs(frontRightPower)+Math.abs(backRightPower));

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);


            // controls servoIntake
            if (gamepad2.left_trigger >.5 ) {
                servoIntake.setPower(1);
            }
            if (gamepad2.right_trigger >.5 ) {
                servoIntake.setPower(-1);
            }

            if (ArmTarget <= 100 - Offset) {
                servoIntake.setPower(-1);
            }

            if (ArmTarget == stack){
                servoIntake.setPower(-1);
            }

            if (gamepad2.left_trigger <.5  && gamepad2.right_trigger <.5 && ArmTarget >100- Offset && ArmTarget != stack){
                servoIntake.setPower(0);
            }

            if (totalPower > 0.1){
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);
            }
            if (totalPower < 0.1){
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }

            // controls ArmTarget

            if (gamepad2.left_bumper) {
                ArmTarget = 70 - Offset; //intaking
            }
            else if (gamepad2.back) {
                ArmTarget = stack; //Cone Stack
            }
            else if (gamepad2.dpad_down) {
                ArmTarget = 1040- Offset; //Low level
            }
            else if (gamepad2.dpad_up) {
                ArmTarget = 1780 - Offset; //Mid level
            }
            else if (gamepad2.right_bumper) {
                ArmTarget = 2550 - Offset; //High level
            }

            // Check if the left joystick button on gamepad 2 is pressed
            boolean leftStickButtonIsPressed = gamepad2.left_stick_button;
            if (leftStickButtonIsPressed && !aButtonWasPressed) {
                // Button was just pressed, decrease the stack by 105 but don't let it go below 70
                stack = Math.max(70- Offset, stack - 105);
            }
            aButtonWasPressed = leftStickButtonIsPressed;

            // Check if the right joystick button on gamepad 2 is pressed
            boolean rightStickButtonIsPressed = gamepad2.right_stick_button;
            if (rightStickButtonIsPressed && !bButtonWasPressed) {
                // Button was just pressed, increase the stack by 105 but don't let it go above 490
                stack = Math.min(490- Offset, stack + 105);
            }
            bButtonWasPressed = rightStickButtonIsPressed;

            //code for manual override adjustments for lift

            boolean ga2y = gamepad2.y;
            boolean ga2a = gamepad2.a;

            if (ga2y && !pGA2Y && ArmTarget<=2550 - Offset) {
                ArmTarget = ArmTarget + 50; //used to be 100
            }
            pGA2Y = ga2y;

            if (ga2a && !pGA2A && ArmTarget>=50 - Offset) {
                ArmTarget = ArmTarget - 50; //used to be 100
            }
            pGA2A = ga2a;

            boolean ga2b = gamepad2.b;
            boolean ga2x = gamepad2.x;

            if (ga2b && !pGA2B) {
                ArmTarget = ArmTarget + 50; //used to be 100
            }
            pGA2B = ga2b;

            if (ga2x && !pGA2X) {
                ArmTarget = ArmTarget - 50; //used to be 100
            }
            pGA2X = ga2x;

            if(stack == 490- Offset){stack_height = 5;}
            if(stack == 385- Offset){stack_height = 4;}
            if(stack == 280- Offset){stack_height = 3;}
            if(stack == 175- Offset){stack_height = 2;}
            if(stack == 70- Offset){stack_height = 1;}




            //stuff for arm position control
            controller = new PIDController(p, i , d);
            int SlidesPos = motorRightLift.getCurrentPosition();

            double pid= controller.calculate(SlidesPos, ArmTarget);

            motorLeftLift.setPower(pid + f);
            motorRightLift.setPower(pid + f);


            telemetry.addData("Stack",stack_height);

            telemetry.addData("pos",SlidesPos );
            telemetry.addData("target", ArmTarget);

            telemetry.update();


            //stops motor after no input
            motorFrontLeft.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackLeft.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontRight.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackRight.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
