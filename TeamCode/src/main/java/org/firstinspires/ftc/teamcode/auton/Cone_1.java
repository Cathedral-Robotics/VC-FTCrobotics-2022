package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous


public class Cone_1 extends LinearOpMode
{OpenCvCamera camera;
    private CRServo servoIntake;

    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
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

        Trajectory traj00 = drive.trajectoryBuilder((startPose))
                .forward(0.001)
                .addDisplacementMarker(() ->{
                    motorLeftLift.setTargetPosition(-300);
                    motorRightLift.setTargetPosition(300);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);


                })
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj00.end())
                .forward(55)
                .build();

        Trajectory traj1_1 = drive.trajectoryBuilder(traj1.end())
                .back(5)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1_1.end())
                .strafeRight(13.5)
                .addDisplacementMarker(2.5, () ->{
                    servoIntake.setPower(-1);

                    motorLeftLift.setTargetPosition(-2500);
                    motorRightLift.setTargetPosition(2500);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })
                .build();



        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(5.50)
                .addDisplacementMarker( () ->{
                    servoIntake.setPower(1);
                })
                .build();

        TrajectorySequence ts3 = drive.trajectorySequenceBuilder(traj3.end())
                .waitSeconds(1.5)
                .build();


        Trajectory traj6 = drive.trajectoryBuilder(ts3.end())
                .back(6)
                .addDisplacementMarker(() ->{
                    servoIntake.setPower(0);
                    motorLeftLift.setTargetPosition(-1050);
                    motorRightLift.setTargetPosition(1050);
                })
                .build();


        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .strafeLeft(13.5)
                .build();

        TrajectorySequence ts7 = drive.trajectorySequenceBuilder(traj7.end())
                .turn(Math.toRadians(94))
                .build();

        Pose2d turnPose_1 = new Pose2d(-71.35,-2.075 ,Math.toRadians(90));

        Trajectory traj8 = drive.trajectoryBuilder(ts7.end())
                .strafeRight(2)
                .build();

        Trajectory One = drive.trajectoryBuilder(ts7.end())
                .forward(21.25)
                .build();

        TrajectorySequence OneOne = drive.trajectorySequenceBuilder(One.end())
                .turn(Math.toRadians(94))
          .addDisplacementMarker(() ->{
          servoIntake.setPower(0);
          motorLeftLift.setTargetPosition(0);
           motorRightLift.setTargetPosition(0);
     })
            .build();


        Trajectory Two = drive.trajectoryBuilder(ts7.end())
                .forward(1)
                .addDisplacementMarker(() ->{
                    servoIntake.setPower(0);
                    motorLeftLift.setTargetPosition(0);
                    motorRightLift.setTargetPosition(0);
                })
                .build();

        Trajectory Three = drive.trajectoryBuilder(ts7.end())
                .back(24)
                .addDisplacementMarker(() ->{
                    servoIntake.setPower(0);
                    motorLeftLift.setTargetPosition(0);
                    motorRightLift.setTargetPosition(0);
                })
                .build();



        TrajectorySequence ts100 = drive.trajectorySequenceBuilder(traj3.end())
                .waitSeconds(30)
                .build();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {




            if(tagOfInterest == null || tagOfInterest.id == LEFT){

                drive.followTrajectory(traj00);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj1);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj1_1);
                drive.followTrajectory(traj2);
                drive.followTrajectory(traj3);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj6);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj7);
                drive.followTrajectorySequence(ts7);
                drive.followTrajectory(traj8);
                drive.followTrajectory(One);
                drive.followTrajectorySequence(OneOne);






                drive.followTrajectorySequence(ts100);
               //

            }else if(tagOfInterest.id == MIDDLE){

                drive.followTrajectory(traj00);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj1);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj1_1);
                drive.followTrajectory(traj2);
                drive.followTrajectory(traj3);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj6);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj7);
                drive.followTrajectorySequence(ts7);
                drive.followTrajectory(traj8);

                drive.followTrajectory(Two);

                drive.followTrajectorySequence(ts100);


            }else{

                drive.followTrajectory(traj00);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj1);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj1_1);
                drive.followTrajectory(traj2);
                drive.followTrajectory(traj3);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj6);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj7);
                drive.followTrajectorySequence(ts7);
                drive.followTrajectory(traj8);

                drive.followTrajectory(Three);

                drive.followTrajectorySequence(ts100);

               //
            }

        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}