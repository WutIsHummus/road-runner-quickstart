package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-35, 62.5,Math.toRadians(-90)))
                .lineToY(33)
                //.afterDisp(0, liftZero)
                //.stopAndAdd(dropPurple)
                .splineToLinearHeading(new Pose2d(-41, 45, Math.toRadians(-90)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-57.5, 36, Math.toRadians(-180)), Math.toRadians(-120))
                /*.afterDisp(4, new SequentialAction(
                        box.moveTo(0.15),
                        gate.moveTo(0.6)
                        ))
                .afterDisp(8, new SequentialAction(
                        midBar.moveToTarget(0, 0.4),
                        box.moveTo(0)
                ))
                 */
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-35, 13), Math.toRadians(0))
                .lineToX(22)
                .splineTo(new Vector2d(45, 35), Math.toRadians(0))
                .lineToX(50, new TranslationalVelConstraint(10))
                .setTangent(Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(22, 13), Math.toRadians(-180))
                .lineToX(-30)
                .splineTo(new Vector2d(-57.5, 12), Math.toRadians(-180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-35, 13), Math.toRadians(0))
                .lineToX(22)
                .splineTo(new Vector2d(45, 35), Math.toRadians(0))
                .lineToX(50, new TranslationalVelConstraint(10))
                .setTangent(Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(22, 13), Math.toRadians(-180))
                .lineToX(-30)
                .splineTo(new Vector2d(-57.5, 12), Math.toRadians(-180))
                .lineToX(22)
                .splineTo(new Vector2d(45, 35), Math.toRadians(0))
                .lineToX(50, new TranslationalVelConstraint(10))
                .setTangent(Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(22, 13), Math.toRadians(-180))
                .lineToX(-30)
                .splineTo(new Vector2d(-57.5, 12), Math.toRadians(-180))
                .lineToX(22)
                .splineTo(new Vector2d(45, 35), Math.toRadians(0))
                .lineToX(50, new TranslationalVelConstraint(10))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}