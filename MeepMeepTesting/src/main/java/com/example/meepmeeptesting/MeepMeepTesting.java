package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.geometry.Vector2d;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(275.4980272676272), 11.65 )
                .setDimensions(13, 17)

                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(new Pose2d(31.50, -21.00, Math.toRadians(225.00)))
                        .setReversed(true)
                        .splineTo(new Vector2d(37.00, -11.50), Math.toRadians(0.00))
                        .setReversed(false)


                        /*drive.trajectorySequenceBuilder(new Pose2d(36.00, -65.00, Math.toRadians(90.00)))
                                .UNSTABLE_addTemporalMarkerOffset(0.00,() -> {})
                                .splineToLinearHeading(new Pose2d(36.00, -7.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(36.00, -10.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                                .splineToLinearHeading(new Pose2d(28.00, -8.00, Math.toRadians(135.00)), Math.toRadians(135.00))
                                .waitSeconds(1)

                                .lineToConstantHeading(new Vector2d(31.50, -10.0))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(41.50, -13.5, Math.toRadians(0)), Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(62.75, -13.5))
                                .waitSeconds(1)

                                .UNSTABLE_addTemporalMarkerOffset(1.33,() -> {})
                                .UNSTABLE_addTemporalMarkerOffset(3.41,() -> {})
                                .lineToConstantHeading(new Vector2d(39.50, -13.5))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(31.50, -12.0, Math.toRadians(225.00)), Math.toRadians(225.00))
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(28.00, -16.50, Math.toRadians(225.00)), Math.toRadians(225.00))
                                .waitSeconds(1)

                                .lineToConstantHeading(new Vector2d(31.50, -12.0))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(41.50, -13.5, Math.toRadians(0)), Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(62.75, -13.5))
                                .waitSeconds(1)

                                .UNSTABLE_addTemporalMarkerOffset(1.33,() -> {})
                                .UNSTABLE_addTemporalMarkerOffset(3.41,() -> {})
                                .lineToConstantHeading(new Vector2d(39.50, -13.5))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(31.50, -12.0, Math.toRadians(225.00)), Math.toRadians(225.00))
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(28.00, -16.50, Math.toRadians(225.00)), Math.toRadians(225.00))
                                .waitSeconds(1)
*/
  /*
                        drive.trajectorySequenceBuilder(new Pose2d(-36.00, -65.00, Math.toRadians(90.00)))
                                .UNSTABLE_addTemporalMarkerOffset(0.00,() -> {})
                                .splineToLinearHeading(new Pose2d(-36.00, -7.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-36.00, -10.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                                .splineToLinearHeading(new Pose2d(-28.00, -8.00, Math.toRadians(45.00)), Math.toRadians(45.00))
                                .waitSeconds(1)

                                .lineToConstantHeading(new Vector2d(-31.50, -10.0))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-41.50, -13.5, Math.toRadians(180.00)), Math.toRadians(180.00))
                                .lineToConstantHeading(new Vector2d(-62.75, -13.5))
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(1.33,() -> {})
                                .UNSTABLE_addTemporalMarkerOffset(3.41,() -> {})
                                .lineToConstantHeading(new Vector2d(-41.50, -13.5))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-31.50, -10.0, Math.toRadians(45.00)), Math.toRadians(45.00))
                                .splineToLinearHeading(new Pose2d(-28.00, -7.50, Math.toRadians(45.00)), Math.toRadians(45.00))
                                .waitSeconds(1)

                                .lineToConstantHeading(new Vector2d(-31.50, -10.0))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-41.50, -13.5, Math.toRadians(180.00)), Math.toRadians(180.00))
                                .lineToConstantHeading(new Vector2d(-62.75, -13.5))
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(1.33,() -> {})
                                .UNSTABLE_addTemporalMarkerOffset(3.41,() -> {})
                                .lineToConstantHeading(new Vector2d(-41.50, -13.5))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-31.50, -10.0, Math.toRadians(45.00)), Math.toRadians(45.00))
                                .splineToLinearHeading(new Pose2d(-28.00, -7.50, Math.toRadians(45.00)), Math.toRadians(45.00))
                                .waitSeconds(1)

                                .setReversed(true)
                                .splineTo(new Vector2d(-35.99, -12.72), Math.toRadians(180.00))
                                .setReversed(false)
                                .splineTo(new Vector2d(-12.00, -12.00), Math.toRadians(0.00))
                                .setReversed(true)
*/

                                .build());





                meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

