package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;




@TeleOp
public class ColdMecanum extends LinearOpMode {
    private CRServo servoIntake;

boolean pGA2Y = false;
boolean pGA2A = false;

    private PIDController controller;
    public static double p = 0.003, i = 0.85, d = 0.014;
    public static double f = 0.078;

    public  static int ArmTarget = 0;

    private DcMotorEx motorLeftLift;
    private DcMotorEx motorRightLift;



    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motorLeftLift = hardwareMap.get(DcMotorEx.class, "motorLeftLift");
        motorRightLift = hardwareMap.get(DcMotorEx.class,"motorRightLift");

        motorLeftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        servoIntake = hardwareMap.get(CRServo.class,"servoIntake");

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

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            if (gamepad2.left_trigger >.5 ) {
                servoIntake.setPower(1);
            }
            if (gamepad2.right_trigger >.5 ) {
                servoIntake.setPower(-1);
            }

            if (ArmTarget == 70) {
                servoIntake.setPower(-1);
            }

            if (gamepad2.left_trigger <.5  && gamepad2.right_trigger <.5 && ArmTarget != 70){
                servoIntake.setPower(0);
            }


            //MECHANISM CODE
//134.4 ticks per rotation
// max extension at 8.7 * 134.4


            if (gamepad2.left_bumper) {
                ArmTarget = 70; //intaking
            }
            else if (gamepad2.dpad_down) {
                ArmTarget = 1750; //Low level
            }
            else if (gamepad2.dpad_up) {
                ArmTarget = 2950; //Mid level
            }
            else if (gamepad2.right_bumper) {
                ArmTarget = 4130; //High level (4400 max)
            }

//code for manual override adjustments for lift

            boolean ga2y = gamepad2.y;
            boolean ga2a = gamepad2.a;

            if (ga2y && !pGA2Y && ArmTarget<=4050 && !gamepad2.dpad_right) {
                ArmTarget = ArmTarget + 50; //used to be 100
            }
            pGA2Y = ga2y;

            if (ga2a && !pGA2A && ArmTarget>=50 && !gamepad2.dpad_right) {
                ArmTarget = ArmTarget - 50; //used to be 100
            }
            pGA2A = ga2a;

            if (ga2y && !pGA2Y && gamepad2.dpad_right) {
                ArmTarget = ArmTarget + 50;
            }

            if (ga2a && !pGA2A && gamepad2.dpad_right) {
                ArmTarget = ArmTarget - 50;
            }


            //stuff for arm position control
            controller = new PIDController(p, i , d);
            int SlidesPos = motorRightLift.getCurrentPosition();

            double pid= controller.calculate(SlidesPos, ArmTarget);

            motorLeftLift.setPower(pid + f);
            motorRightLift.setPower(pid + f);

            telemetry.addData("pos",SlidesPos );
            telemetry.addData("target", ArmTarget);
            telemetry.update();


            telemetry.update();


            //stops motor after no input
            motorFrontLeft.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackLeft.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontRight.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackRight.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
