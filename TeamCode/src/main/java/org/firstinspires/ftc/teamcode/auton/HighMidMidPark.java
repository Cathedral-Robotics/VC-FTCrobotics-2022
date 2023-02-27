package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous

public class HighMidMidPark extends LinearOpMode
{OpenCvCamera camera;
    private CRServo servoIntake;
/*
    private PIDController controller;
    public static double p = 0.0086, i = 0.9, d = 0.00023;
    public static double f = 0.073;

    public static int ArmTarget = 0;

*/

    private DcMotorEx motorLeftLift;
    private DcMotorEx motorRightLift;




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

        // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servoIntake = hardwareMap.get(CRServo.class, "servoIntake");

        motorLeftLift.setTargetPosition(0);
        motorRightLift.setTargetPosition(0);
        motorLeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int ArmTarget = 0;


        TrajectorySequence NewPreload = drive.trajectorySequenceBuilder(new Pose2d(36.00, -65.00, Math.toRadians(90.00)))
                .splineToSplineHeading(new Pose2d(36.00, -50.00, Math.toRadians(155.00)), Math.toRadians(90.00))
                .splineToSplineHeading(new Pose2d(36.00, -12.00, Math.toRadians(135.00)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(28, -8.75, Math.toRadians(135.00)), Math.toRadians(135.00))
                .addDisplacementMarker(38, () ->{
                    motorLeftLift.setTargetPosition(-2560);
                    motorRightLift.setTargetPosition(2560);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })
                .build();
        drive.setPoseEstimate(NewPreload.start());


        TrajectorySequence Preload = drive.trajectorySequenceBuilder(new Pose2d(36.00, -65.00, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(0.00,() -> {})
                .splineToLinearHeading(new Pose2d(36.00, -7.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36.00, -10.00, Math.toRadians(90.00)), Math.toRadians(90.00))

                .splineToLinearHeading(new Pose2d(30.5, -7.75, Math.toRadians(135.00)), Math.toRadians(135.00))

                .addDisplacementMarker(48, () ->{
                    motorLeftLift.setTargetPosition(-2560);
                    motorRightLift.setTargetPosition(2560);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })

                .build();
        drive.setPoseEstimate(Preload.start());

        TrajectorySequence tsdrop = drive.trajectorySequenceBuilder(Preload.end())
                .addTemporalMarker(.030, () ->{
                    servoIntake.setPower(1);

                })
                .waitSeconds(.85)
                .build();

        TrajectorySequence tsdroplong = drive.trajectorySequenceBuilder(Preload.end())
                .addTemporalMarker(.030, () ->{
                    servoIntake.setPower(1);

                })
                .waitSeconds(1.4)
                .build();

        TrajectorySequence Cone_Pickup = drive.trajectorySequenceBuilder(new Pose2d(30.5, -7.75, Math.toRadians(135.00)))
                .lineToConstantHeading(new Vector2d(32.5, -11))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(41.50, -13.5, Math.toRadians(0)), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(64.75, -13.75))
                .addDisplacementMarker(5, () ->{
                    motorLeftLift.setTargetPosition(-700);
                    motorRightLift.setTargetPosition(700);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                    servoIntake.setPower(0);
                })
                .build();

        TrajectorySequence Cone_Pickup_2 = drive.trajectorySequenceBuilder(new Pose2d(28.00, -16.50, Math.toRadians(225.00)))
                .lineToConstantHeading(new Vector2d(31.50, -12.0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(41.50, -13.5, Math.toRadians(0)), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(64.75, -13.75))
                .addDisplacementMarker(5, () ->{
                    motorLeftLift.setTargetPosition(-700);
                    motorRightLift.setTargetPosition(700);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                    servoIntake.setPower(0);
                })
                .build();

        TrajectorySequence New_Cone_Pickup_2 = drive.trajectorySequenceBuilder(new Pose2d(28.00, -16.50, Math.toRadians(225.00)))
                .lineToConstantHeading(new Vector2d(35.50, -10.0))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(41.50, -13.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(64.750, -13.750, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(5, () ->{
                    motorLeftLift.setTargetPosition(-700);
                    motorRightLift.setTargetPosition(700);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                    servoIntake.setPower(0);
                })
                .build();



        TrajectorySequence tspickup_1 = drive.trajectorySequenceBuilder(Cone_Pickup.end())
                .addTemporalMarker(.0010, () ->{
                    motorLeftLift.setTargetPosition(-490);
                    motorRightLift.setTargetPosition(490);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                    servoIntake.setPower(-1);
                })
                .addTemporalMarker(.75, () ->{
                    motorLeftLift.setTargetPosition(-730);
                    motorRightLift.setTargetPosition(730);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })
                .addTemporalMarker(.9, () ->{
                    servoIntake.setPower(0);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence tspickup_2 = drive.trajectorySequenceBuilder(Cone_Pickup.end())
                .addTemporalMarker(.0010, () ->{
                    motorLeftLift.setTargetPosition(-385);
                    motorRightLift.setTargetPosition(385);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                    servoIntake.setPower(-1);
                })
                .addTemporalMarker(.75, () ->{
                    motorLeftLift.setTargetPosition(-730);
                    motorRightLift.setTargetPosition(730);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })
                .addTemporalMarker(.99, () ->{
                    servoIntake.setPower(0);
                })
                .waitSeconds(1)
                .build();



TrajectorySequence Placement = drive.trajectorySequenceBuilder(new Pose2d(64.75, -13, Math.toRadians(0)))
        .UNSTABLE_addTemporalMarkerOffset(1.33,() -> {})
        .UNSTABLE_addTemporalMarkerOffset(3.41,() -> {})
        .lineToConstantHeading(new Vector2d(39.50, -13.5))
        .setReversed(true)
        .splineToLinearHeading(new Pose2d(35.50, -17.0, Math.toRadians(225.00)), Math.toRadians(225.00)) //was 33.5
        .setReversed(false)
        .splineToLinearHeading(new Pose2d(31.50, -22.0, Math.toRadians(225.00)), Math.toRadians(225.00),
                SampleMecanumDrive.getVelocityConstraint(35, 270, 11.65),
                SampleMecanumDrive.getAccelerationConstraint(35)

                )
                                .addDisplacementMarker(21.25, () ->{
                                    motorLeftLift.setTargetPosition(-1850);
                                    motorRightLift.setTargetPosition(1850);
                                    motorLeftLift.setPower(.75);
                                    motorRightLift.setPower(.75);
                                })
                                .build();

        TrajectorySequence Park_1 = drive.trajectorySequenceBuilder(new Pose2d(31.50, -21.00, Math.toRadians(225.00)))
                .setReversed(true)
                .splineTo(new Vector2d(38.00, -13.50), Math.toRadians(0.00))
                .setReversed(false)
                .lineToConstantHeading(new Vector2d(14.125, -12.5))


                .addDisplacementMarker(4.25, () ->{
                    motorLeftLift.setTargetPosition(-0);
                    motorRightLift.setTargetPosition(0);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                    servoIntake.setPower(0);
                })

                .build();

        TrajectorySequence Park_2 = drive.trajectorySequenceBuilder(new Pose2d(31.50, -21.00, Math.toRadians(225.00)))
                .setReversed(true)
                .splineTo(new Vector2d(40.00, -12.50), Math.toRadians(0.00))
                .setReversed(false)

                .addDisplacementMarker(4.25, () ->{
                    motorLeftLift.setTargetPosition(-0);
                    motorRightLift.setTargetPosition(0);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                    servoIntake.setPower(0);
                })

                .build();

        TrajectorySequence Park_3 = drive.trajectorySequenceBuilder(new Pose2d(31.50, -21.00, Math.toRadians(225.00)))
                .setReversed(true)
                .splineTo(new Vector2d(40.00, -12.50), Math.toRadians(0.00))
                .setReversed(true)
                .splineTo(new Vector2d(62.00, -12.50), Math.toRadians(0.00))


                .addDisplacementMarker(4.25, () ->{
                    motorLeftLift.setTargetPosition(-0);
                    motorRightLift.setTargetPosition(0);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                    servoIntake.setPower(0);
                })

                .build();





        TrajectorySequence ts100 = drive.trajectorySequenceBuilder(Preload.end())
                .waitSeconds(3000)
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
/*
            controller = new PIDController(p, i , d);
            int SlidesPos = motorRightLift.getCurrentPosition();

            double pid= controller.calculate(SlidesPos, ArmTarget);

            motorLeftLift.setPower(pid + f);
            motorRightLift.setPower(pid + f);

            telemetry.addData("pos",SlidesPos );
            telemetry.addData("target", ArmTarget);
            telemetry.update();


            telemetry.update();
*/
            if(tagOfInterest == null || tagOfInterest.id == LEFT){


                drive.followTrajectorySequence(NewPreload);
                drive.followTrajectorySequence(tsdroplong);

                drive.followTrajectorySequence(Cone_Pickup);
                drive.followTrajectorySequence(tspickup_1);
                drive.followTrajectorySequence(Placement);
                drive.followTrajectorySequence(tsdrop);

                drive.followTrajectorySequence(Cone_Pickup_2);
                drive.followTrajectorySequence(tspickup_2);
                drive.followTrajectorySequence(Placement);
                drive.followTrajectorySequence(tsdrop);
                drive.followTrajectorySequence(Park_1);




                drive.followTrajectorySequence(ts100);



                //

            }else if(tagOfInterest.id == MIDDLE){

                //
                drive.followTrajectorySequence(Preload);
                drive.followTrajectorySequence(tsdroplong);

                drive.followTrajectorySequence(Cone_Pickup);
                drive.followTrajectorySequence(tspickup_1);
                drive.followTrajectorySequence(Placement);
                drive.followTrajectorySequence(tsdrop);

                drive.followTrajectorySequence(Cone_Pickup_2);
                drive.followTrajectorySequence(tspickup_2);
                drive.followTrajectorySequence(Placement);
                drive.followTrajectorySequence(tsdrop);
                drive.followTrajectorySequence(Park_2);

            }else{

                drive.followTrajectorySequence(Preload);
                drive.followTrajectorySequence(tsdroplong);

                drive.followTrajectorySequence(Cone_Pickup);
                drive.followTrajectorySequence(tspickup_1);
                drive.followTrajectorySequence(Placement);
                drive.followTrajectorySequence(tsdrop);

                drive.followTrajectorySequence(Cone_Pickup_2);
                drive.followTrajectorySequence(tspickup_2);
                drive.followTrajectorySequence(Placement);
                drive.followTrajectorySequence(tsdrop);
                drive.followTrajectorySequence(Park_3);


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