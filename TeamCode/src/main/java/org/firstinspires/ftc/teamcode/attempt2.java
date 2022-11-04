package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name="attempt2")
public class FSMExample extends OpMode {
    // An Enum is used to represent lift states.
    // (This is one thing enums are designed to do)
    public enum LiftState {
        LIFT_PICKUP,
        LIFT_LOW,
        LIFT_MEDIUM,
        LIFT_HIGH,
    };

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    LiftState liftState = LiftState.LIFT_PICKUP;

    // Some hardware access boilerplate; these would be initialized in init()
    // the lift motor, it's in RUN_TO_POSITION mode
    public DcMotor liftMotor;

    // the dump servo
    public Servo liftDump;
    // used with the dump servo, this will get covered in a bit
    ElapsedTime liftTimer = new ElapsedTime();

    final double DUMP_IDLE; // the idle position for the dump servo
    final double DUMP_DEPOSIT; // the dumping position for the dump servo

    // the amount of time the dump servo takes to activate in seconds
    final double DUMP_TIME;

    final int LIFT_LOW; // the low encoder position for the lift
    final int LIFT_HIGH; // the high encoder position for the lift

    public void init() {
        liftTimer.reset();

        // hardware initilization code
    }

    public void loop() {
        switch (liftState) {
            case LiftState.LIFT_PICKUP:
                // Waiting for some input
                if (gamepad1.x) {
                    // x is pressed, start extending
                    liftMotor.setPosition(LIFT_HIGH);
                    liftState = LiftState.LIFT_LOW;
                }
                break;
            case LiftState.LIFT_LOW:
                // check if the left has finished extending,
                // otherwise do nothing.
                if (Math.abs(liftMotor.getPosition() - LIFT_HIGH) < 10) {
                    // our threshold is within
                    // 10 encoder ticks of our target.
                    // this is pretty arbitrary, and would have to be
                    // tweaked for each robot.

                    // set the lift dump to dump
                    liftDump.setPosition(DUMP_DEPOSIT);

                    liftTimer.reset();
                    liftState = LiftState.LIFT_MEDIUM;
                }
                break;
            case LiftState.LIFT_MEDIUM:
                if (liftTimer.seconds() >= DUMP_TIME) {
                    // The robot waited long enough, time to start
                    // retracting the lift
                    liftDump.setPosition(DUMP_IDLE);
                    liftMotor.setPosition(LIFT_LOW);
                    liftState = LiftState. LIFT_HIGH;
                }
                break;
            case LiftState.LIFT_RETRACT:
                if (Math.abs(liftMotor.getPosition() - LIFT_LOW) < 10) {
                    liftState = LiftState.LIFT_PICKUP;
                }
                break;
            default:
                // should never be reached, as liftState should never be null
                liftState = LiftState.LIFT_PICKUP;
        }
    }

    // small optimization, instead of repeating ourselves in each
    // lift state case besides LIFT_PICKUP for the cancel action,
    // it's just handled here
      if (gamepad1.y && liftState != LiftState.LIFT_PICKUP) {
        liftState = LiftState.LIFT_PICKUP;
    }

    // mecanum drive code goes here
    // But since none of the stuff in the switch case stops
    // the robot, this will always run!
    updateDrive(gamepad1, gamepad2);
}
}