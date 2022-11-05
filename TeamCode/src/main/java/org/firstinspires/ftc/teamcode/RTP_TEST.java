package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp

public class RTP_TEST extends LinearOpMode {


    @Override
    public void runOpMode() {
        DcMotor motorLeftLift = hardwareMap.dcMotor.get("motorLeftLift");
        DcMotor motorRightLift = hardwareMap.dcMotor.get("motorRightLift");


        motorLeftLift.setTargetPosition(0);
        motorRightLift.setTargetPosition(0);
        motorLeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int ArmTarget = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //MECHANISM CODE
//134.4 ticks per rotation
// max extension at 8.7 * 134.4


            if (gamepad2.left_bumper) {
                ArmTarget = 3750; //intaking
            }
            else if (gamepad2.dpad_down) {
                ArmTarget = 3900; //Low level
            }
            else if (gamepad2.dpad_up) {
                ArmTarget = 4050; //Mid level
            }
            else if (gamepad2.right_bumper) {
                ArmTarget = 4400; //High level
            }

            //stuff for arm position control
            motorLeftLift.setTargetPosition(-1*ArmTarget);
            motorRightLift.setTargetPosition(ArmTarget);
            motorLeftLift.setPower(.5);
            motorRightLift.setPower(.5);

            telemetry.addData("Left Lift Position", motorLeftLift.getCurrentPosition());
            telemetry.addData("Right Lift Position", motorRightLift.getCurrentPosition());

            telemetry.update();
        }
    }
}
