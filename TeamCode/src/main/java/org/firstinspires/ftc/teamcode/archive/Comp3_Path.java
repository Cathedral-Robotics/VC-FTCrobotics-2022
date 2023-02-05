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

@Autonomous
@Disabled

public class Comp3_Path extends LinearOpMode {
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
                .back(1)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(8)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(50)
                .addDisplacementMarker(36, () ->{
                    servoIntake.setPower(0);

                    motorLeftLift.setTargetPosition(-4100);
                    motorRightLift.setTargetPosition(4100);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
        .strafeLeft(16)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .back(4.25)
                .addDisplacementMarker(() ->{
                    servoIntake.setPower(1);
                })
                .build();

        TrajectorySequence ts5 = drive.trajectorySequenceBuilder(traj5.end())
                .waitSeconds(1)
                .build();


        Trajectory traj6 = drive.trajectoryBuilder(ts5.end())
                .forward(3.75)
                .addDisplacementMarker(() ->{
                    servoIntake.setPower(0);
                    motorLeftLift.setTargetPosition(-1050);
                    motorRightLift.setTargetPosition(1050);
                })
                .build();


        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .strafeRight(16)
                .build();

        TrajectorySequence ts7 = drive.trajectorySequenceBuilder(traj7.end())
                .turn(Math.toRadians(94))
                .build();

        Pose2d turnPose_1 = new Pose2d(-71.35,-2.075 ,Math.toRadians(90));

        Trajectory traj8 = drive.trajectoryBuilder((turnPose_1))
                .back(26)
                .addDisplacementMarker(() ->{
                    servoIntake.setPower(-1);
                    motorLeftLift.setTargetPosition(-700);
                    motorRightLift.setTargetPosition(700);
                })
                .build();

        TrajectorySequence ts8 = drive.trajectorySequenceBuilder(traj8.end())
                .waitSeconds(1)
                .build();


        Trajectory traj9 = drive.trajectoryBuilder(ts8.end())
                .forward(26)
                .addTemporalMarker(0.01, () ->{
                    servoIntake.setPower(0);
                    motorLeftLift.setTargetPosition(-1200);
                    motorRightLift.setTargetPosition(1200);
                })
                .build();

        TrajectorySequence ts9 = drive.trajectorySequenceBuilder(traj9.end())
                .turn(Math.toRadians(-94))
                .build();

        Pose2d turnPose_2 = new Pose2d(-51,-6 ,Math.toRadians(0));


        Trajectory traj10 = drive.trajectoryBuilder((turnPose_2))
                .strafeLeft(15.5)
                .addDisplacementMarker(.01, () ->{
                    servoIntake.setPower(0);

                    motorLeftLift.setTargetPosition(-4100);
                    motorRightLift.setTargetPosition(4100);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })
                .build();

        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .back(3.25)
                .addDisplacementMarker(() ->{
                    servoIntake.setPower(1);
                })
                .build();

        TrajectorySequence ts11 = drive.trajectorySequenceBuilder(traj11.end())
                .waitSeconds(1.5)
                .build();

        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .forward(3.5)
                .addDisplacementMarker(() ->{
                    motorLeftLift.setTargetPosition(0);
                    motorRightLift.setTargetPosition(0);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                    servoIntake.setPower(0);
                })
                .build();



        TrajectorySequence ts100 = drive.trajectorySequenceBuilder(traj4.end())
                .waitSeconds(30)
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
            drive.followTrajectory(traj3);
            drive.followTrajectory(traj4);
            drive.followTrajectory(traj5);
            drive.followTrajectorySequence(ts5);
            drive.followTrajectory(traj6);
            drive.followTrajectory(traj7);
            drive.followTrajectorySequence(ts7);
            drive.followTrajectory(traj8);
            drive.followTrajectorySequence(ts8);
            drive.followTrajectory(traj9);
            drive.followTrajectorySequence(ts9);
            drive.followTrajectory(traj10);
            drive.followTrajectory(traj11);
            drive.followTrajectorySequence(ts11);
            drive.followTrajectory(traj12);




        drive.followTrajectorySequence(ts100);




    }
}