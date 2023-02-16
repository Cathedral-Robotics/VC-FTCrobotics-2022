package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous

public class Spline_Park_1PLUS2 extends LinearOpMode
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

/*
       TrajectorySequence Preload = drive.trajectorySequenceBuilder(new Pose2d(-36, -65, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.15, -23.87), Math.toRadians(74.20))
                .splineTo(new Vector2d(-24, -9), Math.toRadians(90))
                .addDisplacementMarker(3.01, () ->{

                    servoIntake.setPower(-1);
                    motorLeftLift.setTargetPosition(-400);
                    motorRightLift.setTargetPosition(400);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })

                .addDisplacementMarker(40.01, () ->{

                    servoIntake.setPower(0);
                    motorLeftLift.setTargetPosition(2600);
                    motorRightLift.setTargetPosition(2600);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })
                .build();
        drive.setPoseEstimate(Preload.start());


        TrajectorySequence Preload = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -65.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.15, -23.87), Math.toRadians(74.21))
                .splineTo(new Vector2d(-24.00, -9.00), Math.toRadians(90.00))

                .addDisplacementMarker(0.01, () ->{

                    servoIntake.setPower(-1);
                    motorLeftLift.setTargetPosition(-400);
                    motorRightLift.setTargetPosition(400);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })

                .addDisplacementMarker(10, () ->{

                    servoIntake.setPower(0);

                })

                .addDisplacementMarker(30, () ->{
                    motorLeftLift.setTargetPosition(-2500);
                    motorRightLift.setTargetPosition(2500);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })

                .addDisplacementMarker( () ->{
                    servoIntake.setPower(1);

                })
                .build();
        drive.setPoseEstimate(Preload.start());


*/

        TrajectorySequence Preload = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -65.00, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(0.00,() -> {})
                .splineToLinearHeading(new Pose2d(-36.00, -7.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-36.00, -10.00, Math.toRadians(90.00)), Math.toRadians(90.00))

                .splineToLinearHeading(new Pose2d(-28.00, -8.00, Math.toRadians(45.00)), Math.toRadians(45.00))

                .addDisplacementMarker(30, () ->{
                    motorLeftLift.setTargetPosition(-2600);
                    motorRightLift.setTargetPosition(2600);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })

                .build();
        drive.setPoseEstimate(Preload.start());

        TrajectorySequence tsdrop = drive.trajectorySequenceBuilder(Preload.end())
                .addTemporalMarker(.030, () ->{
                    servoIntake.setPower(1);

                })
                .waitSeconds(1)
                .build();

        TrajectorySequence Cone_Pickup = drive.trajectorySequenceBuilder(new Pose2d(-28.00, -8.00, Math.toRadians(45.00)))
                .lineToConstantHeading(new Vector2d(-31.50, -10.0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-41.50, -13.5, Math.toRadians(180.00)), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(-62.75, -13.5))
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





TrajectorySequence Placement = drive.trajectorySequenceBuilder(new Pose2d(-62.75, -13.50, Math.toRadians(180.00)))
        .UNSTABLE_addTemporalMarkerOffset(1.33,() -> {})
        .UNSTABLE_addTemporalMarkerOffset(3.41,() -> {})
        .lineToConstantHeading(new Vector2d(-41.50, -13.5))
        .setReversed(true)
        .splineToLinearHeading(new Pose2d(-31.50, -10.0, Math.toRadians(45.00)), Math.toRadians(45.00))
        .splineToLinearHeading(new Pose2d(-28.00, -7.50, Math.toRadians(45.00)), Math.toRadians(45.00),
                SampleMecanumDrive.getVelocityConstraint(30, 270, 11.65),
                SampleMecanumDrive.getAccelerationConstraint(15)
                )
                                .addDisplacementMarker(21.25, () ->{
                                    motorLeftLift.setTargetPosition(-2550);
                                    motorRightLift.setTargetPosition(2550);
                                    motorLeftLift.setPower(.75);
                                    motorRightLift.setPower(.75);
                                })
                                .build();


        TrajectorySequence Park_1 = drive.trajectorySequenceBuilder(new Pose2d(-28.00, -7.50, Math.toRadians(45.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-59.99, -12.72), Math.toRadians(180.00))
                .addDisplacementMarker(5, () ->{
                    motorLeftLift.setTargetPosition(0);
                    motorRightLift.setTargetPosition(0);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })
                .build();

        TrajectorySequence Park_2 = drive.trajectorySequenceBuilder(new Pose2d(-28.00, -7.50, Math.toRadians(45.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-35.99, -12.72), Math.toRadians(180.00))
                .addDisplacementMarker(5, () ->{
                    motorLeftLift.setTargetPosition(0);
                    motorRightLift.setTargetPosition(0);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })
                .build();

        TrajectorySequence Park_3 = drive.trajectorySequenceBuilder(new Pose2d(-28.00, -7.50, Math.toRadians(45.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-35.99, -12.72), Math.toRadians(180.00))
                .setReversed(false)
                .splineTo(new Vector2d(-12.00, -12.00), Math.toRadians(0.00))
                .setReversed(true)
                .addDisplacementMarker(5, () ->{
                    motorLeftLift.setTargetPosition(0);
                    motorRightLift.setTargetPosition(0);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })
                .build();


/*
        TrajectorySequence Placement = drive.trajectorySequenceBuilder(new Pose2d(-62.750, -13.50, Math.toRadians(180.00)))
                .UNSTABLE_addTemporalMarkerOffset(1.33,() -> {})
                .UNSTABLE_addTemporalMarkerOffset(3.41,() -> {})
                .lineToConstantHeading(new Vector2d(-41.50, -13.5))
                .turn(Math.toRadians(-135))
                .splineTo(new Vector2d(-28.00, -7.50), Math.toRadians(45.00))
                .addDisplacementMarker(21.25, () ->{
                    motorLeftLift.setTargetPosition(-2550);
                    motorRightLift.setTargetPosition(2550);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })
                .build();

TrajectorySequence Placement = drive.trajectorySequenceBuilder(new Pose2d(-62.75, -13.50, Math.toRadians(180.00)))
        .lineTo(new Vector2d(-39.50, -13.50))
        .setReversed(true)
        .splineToLinearHeading(new Pose2d(-28.00, -7.50, Math.toRadians(45.00)), Math.toRadians(45.00))
        .build();


        TrajectorySequence Placement = drive.trajectorySequenceBuilder(new Pose2d(-62.75, -13.50, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-50.00, -13.50))
                .splineToLinearHeading(new Pose2d(-28.00, -7.50, Math.toRadians(45.00)), Math.toRadians(45.00))
                .addDisplacementMarker(21.25, () ->{
                    motorLeftLift.setTargetPosition(-2550);
                    motorRightLift.setTargetPosition(2550);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })
                .build();

         TrajectorySequence Placement = drive.trajectorySequenceBuilder(new Pose2d(-62.75, -13.50, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-47.30, -14.77))
                .lineToLinearHeading(new Pose2d(-34.85, -14.98, Math.toRadians(60.00)))
                .splineTo(new Vector2d(-28.00, -7.50), Math.toRadians(45.00))
                .addDisplacementMarker(21.25, () ->{
                    motorLeftLift.setTargetPosition(-2550);
                    motorRightLift.setTargetPosition(2550);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })
                .build();











        TrajectorySequence Placement = drive.trajectorySequenceBuilder(new Pose2d(-62.00, -12.50, Math.toRadians(180.00)))
                .UNSTABLE_addTemporalMarkerOffset(1.33,() -> {})
                .lineToLinearHeading(new Pose2d(-40.64, -16.38, Math.toRadians(150.00)))
                .splineToSplineHeading(new Pose2d(-28.00, -8.00, Math.toRadians(45.00)), Math.toRadians(45.00))
                .build();



        TrajectorySequence Cone_Pickup = drive.trajectorySequenceBuilder(new Pose2d(-24.00, -9.00, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(-24.00, -19.0), Math.toRadians(90.00))
                .addDisplacementMarker(5, () ->{
                    servoIntake.setPower(0);
                    motorLeftLift.setTargetPosition(-700);
                    motorRightLift.setTargetPosition(700);
                    motorLeftLift.setPower(.75);
                    motorRightLift.setPower(.75);
                })

                .splineTo(new Vector2d(-36.00, -13.50), Math.toRadians(180.00))
                .splineTo(new Vector2d(-62.00, -12.50), Math.toRadians(180.00))
                .build();

        TrajectorySequence Cone_Placement = drive.trajectorySequenceBuilder(new Pose2d(-62.00, -12.50, Math.toRadians(180.00)))
                .splineToConstantHeading(new Vector2d(-35.00, -12.50), Math.toRadians(180.00))
                .splineTo(new Vector2d(-27.75, -6.75), Math.toRadians(45.00))
                .build();

        TrajectorySequence Cone_Pickup_2 = drive.trajectorySequenceBuilder(new Pose2d(-27.75, -6.75, Math.toRadians(45.00)))
                .splineToConstantHeading(new Vector2d(-36.00, -13.50), Math.toRadians(45.00),
                        SampleMecanumDrive.getVelocityConstraint(30, 270, 11.65),
                        SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .splineTo(new Vector2d(-62.00, -13.0), Math.toRadians(180.00),
                        SampleMecanumDrive.getVelocityConstraint(30, 270, 11.65),
                        SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .build();

*/













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


                drive.followTrajectorySequence(Preload);
                drive.followTrajectorySequence(tsdrop);

                drive.followTrajectorySequence(Cone_Pickup);
                drive.followTrajectorySequence(tspickup_1);
                drive.followTrajectorySequence(Placement);
                drive.followTrajectorySequence(tsdrop);

                drive.followTrajectorySequence(Cone_Pickup);
                drive.followTrajectorySequence(tspickup_2);
                drive.followTrajectorySequence(Placement);
                drive.followTrajectorySequence(tsdrop);

                drive.followTrajectorySequence(Park_1);

                drive.followTrajectorySequence(ts100);



                //

            }else if(tagOfInterest.id == MIDDLE){

                //

            }else{
                drive.followTrajectorySequence(Preload);
                drive.followTrajectorySequence(tsdrop);

                drive.followTrajectorySequence(Cone_Pickup);
                drive.followTrajectorySequence(tspickup_1);
                drive.followTrajectorySequence(Placement);
                drive.followTrajectorySequence(tsdrop);

                drive.followTrajectorySequence(Cone_Pickup);
                drive.followTrajectorySequence(tspickup_2);
                drive.followTrajectorySequence(Placement);
                drive.followTrajectorySequence(tsdrop);

                drive.followTrajectorySequence(Park_3);

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