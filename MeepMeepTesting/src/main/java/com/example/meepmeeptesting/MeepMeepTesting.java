package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // Initialize visualizer with window size
        MeepMeep meepMeep = new MeepMeep(650); // Default is 800

        // Change these based on specs of robot and per auto
        final double robotWidth = 17, robotHeight = 14.5;
        final Pose2d startingPose = new Pose2d(38, 56, Math.toRadians(180));

        // Extra calculations for positions
        final double alterationToPushPieces = robotWidth / 2 + 1;

        // Note: All respective units are in inches and seconds
        // Create digitized robot
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(robotWidth, robotHeight) // default is 18x18
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // Create actions for auto
        myBot.runAction(myBot.getDrive().actionBuilder(startingPose)
                        .splineTo(new Vector2d(-34, 38), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-50 + alterationToPushPieces, 10), Math.toRadians(-90))
                        .lineToY(50)
                        .splineToConstantHeading(new Vector2d(-50 + alterationToPushPieces, 10), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-60 + alterationToPushPieces, 10), Math.toRadians(-90))
                        .lineToY(50)
                        .splineToConstantHeading(new Vector2d(-60 + alterationToPushPieces, 10), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-70 + alterationToPushPieces, 10), Math.toRadians(-90))
                        .lineToY(50)
                        .splineToConstantHeading(new Vector2d(-70 + alterationToPushPieces, 10), Math.toRadians(-90))
                        .splineTo(new Vector2d(-34, 10), 0)
                .build());

        // Set custom background for visualizer
        setCustomBackground(meepMeep, "into-the-deep-1.png");

        // Start visualizer
        meepMeep
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }


    // Sets the background of visualizer to that of file with filename in the customBackgrounds folder.
    // Returns true if successfully set, false if default background was used
    private static boolean setCustomBackground(MeepMeep meepMeep, String filename) {
        Image img = null;
        try {
            img = ImageIO.read(new File("MeepMeepTesting/src/main/java/com/example/meepmeeptesting/customBackgrounds/" + filename));
            meepMeep.setBackground(img);
            return true;
        }
        catch(IOException e) {
            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL);
            return false;
        }
    }
}