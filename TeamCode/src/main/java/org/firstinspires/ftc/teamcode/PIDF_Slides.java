package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
@Disabled
public class PIDF_Slides extends OpMode{

    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public  static int ArmTarget = 0;

    private DcMotorEx motorLeftLift;
    private DcMotorEx motorRightLift;


    @Override
    public void init(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motorLeftLift = hardwareMap.get(DcMotorEx.class, "motorLeftLift");
        motorRightLift = hardwareMap.get(DcMotorEx.class,"motorRightLift");

        motorLeftLift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        controller = new PIDController(p, i , d);
                int SlidesPos = motorRightLift.getCurrentPosition();

        double pid= controller.calculate(SlidesPos, ArmTarget);

        motorLeftLift.setPower(pid + f);
        motorRightLift.setPower(pid + f);

        telemetry.addData("pos",SlidesPos );
        telemetry.addData("target", ArmTarget);
        telemetry.update();

    }

}
