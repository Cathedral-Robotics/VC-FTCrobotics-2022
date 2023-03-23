package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous



public class Right_1PLUS2 extends LinearOpMode{

    enum State {
        PRELOAD,
        TSDROP_1,
        CONE_PICKUP_1,
        TSPICKUP_1,
        PLACEMENT_1,
        TSDROP_2,
        CONE_PICKUP_2,
        TSPICKUP_2,
        PLACEMENT_2,
        TSDROP_3,
        PARK_1,
        PARK_2,
        PARK_3,
        IDLE            // Our bot will enter the IDLE state when done
    }
    // We define the current state we're on
    // Default to IDLE
    AsyncFollowing.State currentState = AsyncFollowing.State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(15, 10, Math.toRadians(90));
    public int ArmTarget = 0;
    public int IntakePower = 0;

    OpenCvCamera camera;

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
    int LEFT = 111;
    int MIDDLE = 222;
    int RIGHT = 333;

    AprilTagDetection tagOfInterest = null;

    ElapsedTime waitTimer1 = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

       Lift lift = new Lift(hardwareMap);
       Intake intake = new Intake(hardwareMap);

        // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TrajectorySequence Preload = drive.trajectorySequenceBuilder(new Pose2d(36.00, -65.00, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(0.00,() -> {})
                .splineToLinearHeading(new Pose2d(36.00, -7.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36.00, -10.00, Math.toRadians(90.00)), Math.toRadians(90.00))

                .splineToLinearHeading(new Pose2d(30.5, -7.75, Math.toRadians(135.00)), Math.toRadians(135.00))
                .addTemporalMarker(.001, () ->{
                    waitTimer1.reset();
                })
                .addDisplacementMarker(42, () ->{
                    ArmTarget = 2600;
                })
                .build();
        drive.setPoseEstimate(Preload.start());


        TrajectorySequence Cone_Pickup = drive.trajectorySequenceBuilder(Preload.end())
                .lineToConstantHeading(new Vector2d(32.5, -11))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(41.50, -13.5, Math.toRadians(0)), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(64.75, -13.75))
                .addDisplacementMarker(5, () ->{
                    ArmTarget = 700;
                    IntakePower=0;
                })
                .build();


        TrajectorySequence Placement = drive.trajectorySequenceBuilder(Cone_Pickup.end())
                .UNSTABLE_addTemporalMarkerOffset(1.33,() -> {})
                .UNSTABLE_addTemporalMarkerOffset(3.41,() -> {})
                .lineToConstantHeading(new Vector2d(39.50, -13.5))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(32.50, -10.75, Math.toRadians(45.00)), Math.toRadians(45.00))
                .splineToLinearHeading(new Pose2d(30.50, -8.75, Math.toRadians(45.00)), Math.toRadians(45.00),
                        SampleMecanumDrive.getVelocityConstraint(30, 270, 11.65),
                        SampleMecanumDrive.getAccelerationConstraint(15)
                )
                .addDisplacementMarker(18.25, () ->{
                    ArmTarget = 2600;
                })
                .build();

        TrajectorySequence tsdrop = drive.trajectorySequenceBuilder(Placement.end())
                .waitSeconds(.5)
                .addTemporalMarker(.050, () ->{
                    IntakePower=1;
                })
                .build();

        TrajectorySequence tspickup_1 = drive.trajectorySequenceBuilder(Cone_Pickup.end())
                .waitSeconds(1)
                .addTemporalMarker(.050, () ->{
                    ArmTarget = 490;
                    IntakePower=-1;
                })
                .addTemporalMarker(.75, () ->{
                    ArmTarget=750;
                })
                .addTemporalMarker(.99, () ->{
                    IntakePower=0;
                })
                .build();

        TrajectorySequence tspickup_2 = drive.trajectorySequenceBuilder(Cone_Pickup.end())
                .waitSeconds(1)
                .addTemporalMarker(.050, () ->{
                    ArmTarget = 385;
                    IntakePower=-1;
                })
                .addTemporalMarker(.75, () ->{
                    ArmTarget=730;
                })
                .addTemporalMarker(.99, () ->{
                    IntakePower=0;
                })
                .build();


        TrajectorySequence Park_2 = drive.trajectorySequenceBuilder(Placement.end())
                .setReversed(true)
                .splineTo(new Vector2d(40.00, -12.50), Math.toRadians(0.00))
                .setReversed(false)
                .addDisplacementMarker(5, () ->{
                   ArmTarget=700;
                   IntakePower=0;
                })
                .build();

        TrajectorySequence Park_3 = drive.trajectorySequenceBuilder(Placement.end())
                .setReversed(true)
                .splineTo(new Vector2d(40.00, -12.50), Math.toRadians(0.00))
                .setReversed(true)
                .splineTo(new Vector2d(62.00, -12.50), Math.toRadians(0.00))
                .addDisplacementMarker(5, () ->{
                    ArmTarget=700;
                    IntakePower=0;
                })
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

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = AsyncFollowing.State.PRELOAD;
        drive.followTrajectorySequenceAsync(Preload);

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

        while (opModeIsActive()){

                switch (currentState) {
                    case PRELOAD:
                        // Check if the drive class isn't busy
                        // `isBusy() == true` while it's following the trajectory
                        // We move on to the next state
                        // Make sure we use the async follow function
                        if (!drive.isBusy()) {                    // Once `isBusy() == false`, the trajectory follower signals that it is finished

                            currentState = AsyncFollowing.State.TSDROP_1;
                            drive.followTrajectorySequenceAsync(tsdrop);
                        }
                        break;
                    case TSDROP_1:
                        // Check if the drive class is busy following the trajectory
                        // Move on to the next state, TURN_1, once finished
                        if (!drive.isBusy()) {
                            currentState = AsyncFollowing.State.CONE_PICKUP_1;
                            drive.followTrajectorySequenceAsync(Cone_Pickup);
                        }
                        break;
                    case CONE_PICKUP_1:
                        // Check if the drive class is busy turning
                        // If not, move onto the next state, TRAJECTORY_3, once finished
                        if (!drive.isBusy()) {
                            currentState = AsyncFollowing.State.TSPICKUP_1;
                            drive.followTrajectorySequenceAsync(tspickup_1);
                        }
                        break;
                    case TSPICKUP_1:
                        // Check if the drive class is busy following the trajectory
                        // If not, move onto the next state, WAIT_1
                        if (!drive.isBusy()) {
                            currentState = AsyncFollowing.State.PLACEMENT_1;
                            drive.followTrajectorySequenceAsync(Placement);
                        }
                        break;
                    case PLACEMENT_1:
                        if (!drive.isBusy()) {
                            currentState = AsyncFollowing.State.TSDROP_2;
                            drive.followTrajectorySequenceAsync(tsdrop);
                        }
                        break;
                    case TSDROP_2:
                        // Check if the drive class is busy following the trajectory
                        // Move on to the next state, TURN_1, once finished
                        if (!drive.isBusy()) {
                            currentState = AsyncFollowing.State.CONE_PICKUP_2;
                            drive.followTrajectorySequenceAsync(Cone_Pickup);
                        }
                        break;
                    case CONE_PICKUP_2:
                        // Check if the drive class is busy turning
                        // If not, move onto the next state, TRAJECTORY_3, once finished
                        if (!drive.isBusy()) {
                            currentState = AsyncFollowing.State.TSPICKUP_2;
                            drive.followTrajectorySequenceAsync(tspickup_2);
                        }
                        break;
                    case TSPICKUP_2:
                        // Check if the drive class is busy following the trajectory
                        // If not, move onto the next state, WAIT_1
                        if (!drive.isBusy()) {
                            currentState = AsyncFollowing.State.PLACEMENT_2;
                            drive.followTrajectorySequenceAsync(Placement);
                        }
                        break;
                    case PLACEMENT_2:
                        if (!drive.isBusy()) {
                            currentState = AsyncFollowing.State.TSDROP_3;
                            drive.followTrajectorySequenceAsync(tsdrop);
                        }
                        break;
                    case TSDROP_3:
                        // Check if the drive class is busy following the trajectory
                        // Move on to the next state, TURN_1, once finished
                        if (!drive.isBusy() && tagOfInterest.id == LEFT && waitTimer1.seconds() >= 27.5) {
                            currentState = AsyncFollowing.State.PARK_1;
                            drive.followTrajectorySequenceAsync(Cone_Pickup);
                        }
                        if (!drive.isBusy() && tagOfInterest.id == MIDDLE && waitTimer1.seconds() >= 27.5) {
                            currentState = AsyncFollowing.State.PARK_2;
                            drive.followTrajectorySequenceAsync(Park_2);
                        }
                        if (!drive.isBusy() && tagOfInterest == null || tagOfInterest.id == RIGHT && waitTimer1.seconds() >= 27.5) {
                            currentState = AsyncFollowing.State.PARK_3;
                            drive.followTrajectorySequenceAsync(Park_3);
                        }
                        break;

                    case PARK_1:
                        // Check if the drive class is busy turning
                        // If not, move onto the next state, TRAJECTORY_3, once finished
                        if (!drive.isBusy()) {
                            currentState = AsyncFollowing.State.IDLE;
                        }
                        break;
                    case PARK_2:
                        // Check if the drive class is busy turning
                        // If not, move onto the next state, TRAJECTORY_3, once finished
                        if (!drive.isBusy()) {
                            currentState = AsyncFollowing.State.IDLE;
                        }
                        break;
                    case PARK_3:
                        // Check if the drive class is busy turning
                        // If not, move onto the next state, TRAJECTORY_3, once finished
                        if (!drive.isBusy()) {
                            currentState = AsyncFollowing.State.IDLE;
                        }
                        break;
                    case IDLE:
                        // Do nothing in IDLE
                        // currentState does not change once in IDLE
                        // This concludes the autonomous program
                        break;
                }

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            lift.update();
            // We update intake continuously in the background, regardless of state
            intake.update();


            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("State", currentState);
            telemetry.addData("Timer", waitTimer1);
            telemetry.update();
        }
    }
    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop


    class Lift {

        private PIDController controller;

        public double p = 0.0086, i = 0.9, d = 0.00023;
        public double f = 0.073;

        //double? or static

        private DcMotorEx motorLeftLift;
        private DcMotorEx motorRightLift;

        public Lift(HardwareMap hardwareMap) {
            /* controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
*/
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            motorLeftLift = hardwareMap.get(DcMotorEx.class, "motorLeftLift");
            motorRightLift = hardwareMap.get(DcMotorEx.class,"motorRightLift");

            motorLeftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public void update() {
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

    class Intake {

        private CRServo servoIntake;

        public Intake(HardwareMap hardwareMap) {
            servoIntake = hardwareMap.get(CRServo.class, "servoIntake");
        }

        public void update() {
            if (IntakePower == 0) {
                servoIntake.setPower(0);
            }
            if (IntakePower == -1) {
                servoIntake.setPower(-1);
            }
            if (IntakePower == 1) {
                servoIntake.setPower(1);
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
