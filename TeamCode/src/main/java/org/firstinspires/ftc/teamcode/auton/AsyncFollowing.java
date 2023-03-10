package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */

@Autonomous(group = "auton")
public class AsyncFollowing extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
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
        CONE_PICKUP_3,
        TSPICKUP_3,
        PLACEMENT_3,
        TSDROP_4,
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(15, 10, Math.toRadians(90));
    public int ArmTarget = 0;
    public int IntakePower = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift & Intake
        Lift lift = new Lift(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        TrajectorySequence Preload = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -65.00, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(0.00,() -> {})
                .splineToLinearHeading(new Pose2d(-36.00, -7.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-36.00, -10.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(-28.00, -8.0, Math.toRadians(45.00)), Math.toRadians(45.00))

                .addDisplacementMarker(30, () ->{
                    ArmTarget = 2600;
                })

                .build();
        drive.setPoseEstimate(Preload.start());


        TrajectorySequence Cone_Pickup = drive.trajectorySequenceBuilder(Preload.end())
                .lineToConstantHeading(new Vector2d(-31.50, -10.0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-41.50, -13.5, Math.toRadians(180.00)), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(-61, -13.5))
                .addDisplacementMarker(5, () ->{
                    ArmTarget = 700;
                })
                .build();


        TrajectorySequence Placement = drive.trajectorySequenceBuilder(Cone_Pickup.end())
                .UNSTABLE_addTemporalMarkerOffset(1.33,() -> {})
                .UNSTABLE_addTemporalMarkerOffset(3.41,() -> {})
                .lineToConstantHeading(new Vector2d(-41.50, -13.5))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-31.50, -10.0, Math.toRadians(45.00)), Math.toRadians(45.00))
                .splineToLinearHeading(new Pose2d(-28.00, -8.0, Math.toRadians(45.00)), Math.toRadians(45.00),
                        SampleMecanumDrive.getVelocityConstraint(30, 270, 11.65),
                        SampleMecanumDrive.getAccelerationConstraint(15)
                )
                .addDisplacementMarker(28.25, () ->{
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

        TrajectorySequence tspickup_3 = drive.trajectorySequenceBuilder(Cone_Pickup.end())
                .waitSeconds(1)
                .addTemporalMarker(.050, () ->{
                    ArmTarget = 280;
                    IntakePower=-1;
                })
                .addTemporalMarker(.75, () ->{
                    ArmTarget=730;
                })
                .addTemporalMarker(.99, () ->{
                    IntakePower=0;
                })
                .build();

        // Define a 1.5 second wait time
        double waitTime1 = 0.75;
        ElapsedTime waitTimer1 = new ElapsedTime();


        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.PRELOAD;
        drive.followTrajectorySequenceAsync(Preload);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case PRELOAD:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {                    // Once `isBusy() == false`, the trajectory follower signals that it is finished

                        currentState = State.TSDROP_1;
                        drive.followTrajectorySequenceAsync(tsdrop);
                    }
                    break;
                case TSDROP_1:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.CONE_PICKUP_1;
                        drive.followTrajectorySequenceAsync(Cone_Pickup);
                    }
                    break;
                case CONE_PICKUP_1:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TSPICKUP_1;
                        drive.followTrajectorySequenceAsync(tspickup_1);
                    }
                    break;
                case TSPICKUP_1:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.PLACEMENT_1;
                        drive.followTrajectorySequenceAsync(Placement);
                    }
                    break;
                case PLACEMENT_1:
                    if (!drive.isBusy()) {
                        currentState = State.TSDROP_2;
                        drive.followTrajectorySequenceAsync(tsdrop);
                    }
                    break;
                case TSDROP_2:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.CONE_PICKUP_2;
                        drive.followTrajectorySequenceAsync(Cone_Pickup);
                    }
                    break;
                case CONE_PICKUP_2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TSPICKUP_2;
                        drive.followTrajectorySequenceAsync(tspickup_2);
                    }
                    break;
                case TSPICKUP_2:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.PLACEMENT_2;
                        drive.followTrajectorySequenceAsync(Placement);
                    }
                    break;
                case PLACEMENT_2:
                    if (!drive.isBusy()) {
                        currentState = State.TSDROP_3;
                        drive.followTrajectorySequenceAsync(tsdrop);
                    }
                    break;
                case TSDROP_3:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.CONE_PICKUP_3;
                        drive.followTrajectorySequenceAsync(Cone_Pickup);
                    }
                    break;
                case CONE_PICKUP_3:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TSPICKUP_3;
                        drive.followTrajectorySequenceAsync(tspickup_3);
                    }
                    break;
                case TSPICKUP_3:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.PLACEMENT_3;
                        drive.followTrajectorySequenceAsync(Placement);
                    }
                    break;
                case PLACEMENT_3:
                    if (!drive.isBusy()) {
                        currentState = State.TSDROP_4;
                        drive.followTrajectorySequenceAsync(tsdrop);
                    }
                    break;

                case TSDROP_4:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

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
            servoIntake = hardwareMap.get(CRServo.class,"servoIntake");
        }

        public void update() {
            if (IntakePower == 0){
                servoIntake.setPower(0);
            }
            if (IntakePower == -1){
                servoIntake.setPower(-1);
            }
            if (IntakePower == 1){
                servoIntake.setPower(1);
            }
        }
    }
}