package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.helpers.Actions.Flopper;
import org.firstinspires.ftc.teamcode.helpers.Actions.Lift;
import org.firstinspires.ftc.teamcode.helpers.Actions.Box;
import org.firstinspires.ftc.teamcode.helpers.Actions.Gate;
import org.firstinspires.ftc.teamcode.helpers.Actions.MidBar;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Blue Auto Far Mid", group="Linear OpMode")
public final class BlueFar extends LinearOpMode {
    private DcMotorEx liftLeft, liftRight, middleBar, flopperMotor;
    private Servo boxServo, armServo;
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        middleBar = hardwareMap.get(DcMotorEx.class, "middleBar");
        boxServo = hardwareMap.servo.get("box");
        armServo = hardwareMap.servo.get("armGate");
        flopperMotor = hardwareMap.get(DcMotorEx.class, "flopper");
        liftLeft.setDirection(DcMotorEx.Direction.REVERSE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up actions
        Flopper flopper = new Flopper(flopperMotor);
        MidBar midBar = new MidBar(middleBar);
        Lift lift = new Lift(liftLeft, liftRight);
        Box box = new Box(boxServo);
        Gate gate = new Gate(armServo);
        Action liftTemp = lift.moveToTarget(-100, 1f); // Lift up so box doesnt get stuck
        Action liftZero = lift.moveToTarget(0, 1f); // Lift zero
        Action dropPurple = flopper.moveToTarget(-300, 0.5f); // Small ration to drop pixel
        Action intake = flopper.moveToTarget(-5000, 1f);// Spin intake a lot

        Pose2d beginPose = new Pose2d(-35, 62.5, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                liftTemp,
                drive.actionBuilder(beginPose)
                        .stopAndAdd(new ParallelAction(
                                gate.moveTo(0.2),
                                box.moveTo(0.15),
                                midBar.moveToTarget(-250, 0.5)
                        ))
                        .lineToY(33)
                        .afterDisp(0, liftZero)
                        //.stopAndAdd(dropPurple)
                        .splineToLinearHeading(new Pose2d(-41, 45, Math.toRadians(-90)), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-57.5, 36, Math.toRadians(-180)), Math.toRadians(-120))
                        .afterDisp(4, new SequentialAction(
                                box.moveTo(0.15),
                                gate.moveTo(0.6)
                                ))
                        .afterDisp(8, new SequentialAction(
                                midBar.moveToTarget(0, 0.4),
                                box.moveTo(0)
                        ))
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
                        .build()
                ));

        /*
        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder( new Pose2d(-35, 62.5, Math.toRadians(-90)))
                                .lineToY(33)
                                .stopAndAdd(dropPurple)
                                .splineToLinearHeading(new Pose2d(-41, 45, Math.toRadians(-90)), Math.toRadians(180))
                                .build(),
                        new ParallelAction(
                                drive.actionBuilder(new Pose2d(-41, 45, Math.toRadians(-90)))
                                        .splineToSplineHeading(new Pose2d(-57.5, 36, Math.toRadians(-180)), Math.toRadians(-120))
                                        .build()
                                // Add move box down action to get from stack
                                // EX: moveLift5
                        ),
                        drive.actionBuilder(new Pose2d(-57.5, 36, Math.toRadians(-180)))
                                //use servo to knock down stack
                                //Move left
                                // Use huskeylens to align
                                .stopAndAdd(intake)
                                           .build(),
                        new ParallelAction(
                                drive.actionBuilder(drive.pose)
                                        .setTangent(Math.toRadians(-90))
                                        .splineToConstantHeading(new Vector2d(-35, 10), Math.toRadians(0))
                                        .build()
                                //Move mid bar back up
                        ),
                        new ParallelAction(
                                drive.actionBuilder(drive.pose)
                                        .lineToX(22)
                                        .build()
                                // Move lift and intake
                        ),
                        new ParallelAction(
                                drive.actionBuilder(drive.pose)
                                        .splineTo(new Vector2d(45, 35), Math.toRadians(0))
                                        //Lift up intake
                                        .build()
                        )
                )
        );
         */

    }

}
