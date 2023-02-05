package org.firstinspires.ftc.teamcode.archive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous

public class New_SameColor extends LinearOpMode {
    private CRServo servoIntake;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor motorLeftLift = hardwareMap.dcMotor.get("motorLeftLift");
        DcMotor motorRightLift = hardwareMap.dcMotor.get("motorRightLift");

        servoIntake = hardwareMap.get(CRServo.class, "servoIntake");

        motorLeftLift.setTargetPosition(0);
        motorRightLift.setTargetPosition(0);
        motorLeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int ArmTarget = 0;

        Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));

        Trajectory traj1 = drive.trajectoryBuilder((startPose))
                .forward(2.5)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(22)
                .addDisplacementMarker(() ->{
                    motorLeftLift.setTargetPosition(-325);
                    motorRightLift.setTargetPosition(325);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                    servoIntake.setPower(1);
                })
                .build();

        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(traj2.end())
                .waitSeconds(1.5)
                .build();


        Trajectory traj3 = drive.trajectoryBuilder(ts2.end())
                .addDisplacementMarker(() ->{
                    servoIntake.setPower(0);
                })
                .forward(24)
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .turn(Math.toRadians(-94))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .back(48)
                .build();

        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .turn(Math.toRadians(94))
                .build();


        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .back(36)
                .addTemporalMarker(.1,() ->{
                    motorLeftLift.setTargetPosition(-900);
                    motorRightLift.setTargetPosition(900);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })
                .build();



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (d
        // .
        // Driver presses PLAY)


        waitForStart();

        if (isStopRequested()) return;



            telemetry.addData("Left Lift Position", motorLeftLift.getCurrentPosition());
            telemetry.addData("Right Lift Position", motorRightLift.getCurrentPosition());
            telemetry.update();

            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
            drive.followTrajectorySequence(ts2);
            drive.followTrajectory(traj3);
            drive.followTrajectorySequence(traj4);
            drive.followTrajectory(traj5);
            drive.followTrajectorySequence(traj6);
            drive.followTrajectory(traj7);




        sleep(100000000);

    }
}