package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helpers.Actions.Flopper;
import org.firstinspires.ftc.teamcode.helpers.VisionSystem;
import java.util.concurrent.TimeUnit;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Alignment", group = "Sensor")
public class PixelTest extends LinearOpMode {
    private VisionSystem visionSystem;
    private DcMotorEx liftLeft, liftRight, middleBar, flopperMotor;
    private Servo boxServo, armServo;
    boolean toggle = false;
    double distance;
    double angle;
    Vector2d Estimate;
    @Override
    public void runOpMode() {
        HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        flopperMotor = hardwareMap.get(DcMotorEx.class, "flopper");
        visionSystem = new VisionSystem(huskyLens, telemetry);
        Pose2d beginPose = new Pose2d(-35, 62.5, Math.toRadians(-90));
        Flopper flopper = new Flopper(flopperMotor);
        flopperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Action intake = flopper.moveToTarget(1000, 1f);// Spin intake a lot
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a)  toggle = false;
            else if (gamepad1.b) toggle = true;

            if (toggle) {
                double theta;
                if (Math.abs(angle) > 2) theta = angle;
                else theta = 0;
                // Calculate target coordinates
                double targetY = drive.pose.position.y - (distance / ((Math.sqrt(2)) * 2 ));
                double targetX = drive.pose.position.x - distance / ((Math.sqrt(2)) * 2 ) *  Math.tan(Math.toRadians(theta));
                telemetry.addData("X", targetX);
                telemetry.addData("Y", targetY);
                if (gamepad1.x){
                    Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(targetX, targetY), Math.toRadians(-90))
                            .afterDisp(0, intake)
                            .build());
                }

            } else {
                HuskyLens.Block[] blocks = visionSystem.getBlocks();
                telemetry.addData("Block count", blocks.length);
                for (HuskyLens.Block block : blocks) {
                    distance = visionSystem.calculateDistance(block.width);
                    angle = visionSystem.calculateHorizontalRotationAngle(block.x, 320);

                    telemetry.addData("Position", block.x);
                    telemetry.addData("Distance", distance * Math.sin(Math.toRadians(45)));
                    telemetry.addData("Angle", angle);
                }
            }

            telemetry.update();
        }
    }
}
