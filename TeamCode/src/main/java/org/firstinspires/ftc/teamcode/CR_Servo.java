package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@TeleOp
@Disabled

public class CR_Servo extends LinearOpMode {
    private CRServo servoIntake;

    @Override
    public void runOpMode() {
    servoIntake = hardwareMap.get(CRServo.class,"servoIntake");


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (d
        // .
        // river presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //MECHANISM CODE

           if (gamepad2.left_trigger >.5 ) {
           servoIntake.setPower(1);
           }
           if (gamepad2.right_trigger >.5 ) {
                servoIntake.setPower(-1);
            }
            if (gamepad2.left_trigger <.5  && gamepad2.right_trigger <.5){
                servoIntake.setPower(0);
            }
            telemetry.update();
        }
    }
}