Package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ServoTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        Servo motorBackRight = hardwareMap.servo ("Servo1");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {