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



                        drive.trajectorySequenceBuilder(new Pose2d(-61.75, -13.5, Math.toRadians(180)))
                                .UNSTABLE_addTemporalMarkerOffset(1.33,() -> {})
                                .UNSTABLE_addTemporalMarkerOffset(3.41,() -> {})
                                .lineToConstantHeading(new Vector2d(-41.50, -13.5))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-31.50, -10.0, Math.toRadians(45.00)), Math.toRadians(45.00))
                                .splineToLinearHeading(new Pose2d(-28.00, -8.0, Math.toRadians(45.00)), Math.toRadians(45.00))


                                .build());





                meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

