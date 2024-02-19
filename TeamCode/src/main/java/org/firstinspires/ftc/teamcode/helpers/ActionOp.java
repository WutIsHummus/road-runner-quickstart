package org.firstinspires.ftc.teamcode.helpers;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.ActionOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.helpers.Actions.Box;
import org.firstinspires.ftc.teamcode.helpers.Actions.Flopper;
import org.firstinspires.ftc.teamcode.helpers.Actions.Gate;
import org.firstinspires.ftc.teamcode.helpers.Actions.Lift;
import org.firstinspires.ftc.teamcode.helpers.Actions.MidBar;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.helpers.PIDFController;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Teleop Field Centric")
@Config
public class ActionOp extends ActionOpMode {


    // Declare a PIDF Controller to regulate heading
    private final PIDFController.PIDCoefficients HEADING_PID_JOYSTICK = new PIDFController.PIDCoefficients(0.2, 0.0, 1);
    private final PIDFController joystickHeadingController = new PIDFController(HEADING_PID_JOYSTICK);
    private final PIDFController.PIDCoefficients PIXEL_PID_JOYSTICK = new PIDFController.PIDCoefficients(0.004, 0.0, 0.0);
    private final PIDFController pixelHeadingController = new PIDFController(PIXEL_PID_JOYSTICK);
    double speed;
    LynxModule CONTROL_HUB;
    LynxModule EXPANSION_HUB;
    boolean fieldCentric = true;
    public MecanumDrive drive;

    List<Action> runningActions = new ArrayList<>();
    final ElapsedTime liftTimer = new ElapsedTime();
    final ElapsedTime loopTime = new ElapsedTime();
    boolean pixelInClaw = false;
    boolean pixelInHook = true;//false;
    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();

    boolean showMotorTelemetry = true;
    boolean showStateTelemetry = true;
    boolean showLoopTimes = true;
    boolean showTelemetryMenu = false;
    boolean showPoseTelemetry = true;
    boolean showCameraTelemetry = false;

    boolean drivingEnabled = true;
    boolean actionRunning = false;
    boolean suspendSet = false;
    private DcMotorEx liftLeft, liftRight, middleBar, flopperMotor;
    private Servo boxServo, armServo;


    @Override
    public void runOpMode() {

        // Enable Bulk Caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        CONTROL_HUB = allHubs.get(0);
        EXPANSION_HUB = allHubs.get(1);

        // RoadRunner Init
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        joystickHeadingController.setInputBounds(-Math.PI, Math.PI);

        // Telemetry Init
        telemetry.setMsTransmissionInterval(50);

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

        waitForStart();

        if (isStopRequested()) return;


        // Run Period

        while (opModeIsActive() && !isStopRequested()) {
            // Reset measured loop time
            loopTime.reset();
            // Reset bulk cache
            allHubs.forEach(LynxModule::clearBulkCache);

            // This lets us do reliable rising edge detection, even if it changes mid loop
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // CONTROLS

            // Gamepad 1
            // Driving Modifiers
            boolean padSlowMode = gamepad1.left_bumper;
            boolean padFastMode = gamepad1.right_bumper;
            boolean padResetPose = gamepad1.dpad_left && !previousGamepad1.dpad_left;

            // Misc/Obscure
            boolean padCameraAutoAim = gamepad1.right_stick_button;

            // Extra Settings
            boolean pad1ExtraSettings = gamepad1.share;
            boolean pad1ExTeamSwitch = gamepad1.dpad_left && !previousGamepad1.dpad_left; // 1 rumble blue, 2 rumble red
            boolean pad1ExToggleFieldCentric = gamepad1.dpad_up && !previousGamepad1.dpad_up;


            // Gamepad 2
            // Presets/Automated

            boolean padHighPreset = gamepad2.y;
            boolean padMidPreset = gamepad2.b;
            boolean padLowPreset = gamepad2.a;

            boolean padClawToggle = (gamepad2.right_bumper && !previousGamepad2.right_bumper); //|| (gamepad1.square && !previousGamepad1.square);

            boolean padShooter = gamepad2.square; //|| gamepad1.square;

            // Manual Control
            double padSlideControl = -gamepad2.left_stick_y;
            double padSlideControlMultiplier = 40;
            /*
            double padArmControl = -gamepad2.right_stick_y;
            double padArmControlMultiplier = 2;

             */

            double padSuspendControl = -gamepad2.right_stick_y;
            double padSuspendControlMultiplier = 1;


            // Misc
            double padGunnerDrive = gamepad2.right_stick_x; // only when right trigger held
            boolean padForceDown = gamepad2.dpad_down && gamepad2.options;
            boolean padMissedHook = gamepad2.dpad_up;
            boolean padAutoPlacer = gamepad2.dpad_down && gamepad2.share;

            boolean padSuspendMode = gamepad2.triangle && !previousGamepad2.triangle;


            // Update the speed
            if (padSlowMode) {
                speed = .35;
            } else if (padFastMode) {
                speed = 1.5;
            } else {
                speed = .8; // prev 0.8
            }
            // especially in driver practice, imu drifts eventually
            // this lets them reset just in case
            if (padResetPose) {
                if (!(PoseStorage.currentTeam == PoseStorage.Team.BLUE)) { // Team is declared and saved there for auto
                    drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(90.0));
                } else {
                    drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(-90.0));
                }
                gamepad1.rumbleBlips(1); // tell the driver it succeeded
            }
            // Second layer
            if (pad1ExtraSettings) {
                if (pad1ExToggleFieldCentric) {
                    fieldCentric = !fieldCentric;
                    if (fieldCentric) {
                        gamepad1.rumbleBlips(2);
                    } else {
                        gamepad1.rumbleBlips(1);
                    }
                }

                if (pad1ExTeamSwitch) {
                    if (PoseStorage.currentTeam == PoseStorage.Team.RED) {
                        gamepad1.rumbleBlips(1);
                        PoseStorage.currentTeam = PoseStorage.Team.BLUE;

                    } else {
                        gamepad1.rumbleBlips(2);
                        PoseStorage.currentTeam = PoseStorage.Team.RED;
                    }
                }
            }

            // Field Centric

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * speed,
                    -gamepad1.left_stick_x * speed
            );

            //Pose2d poseEstimate = drive.pose;
            double rotationAmount = -drive.pose.heading.log(); // Rotation2d.log() makes it into a double in radians.
            if (fieldCentric && !padCameraAutoAim) {
                if (PoseStorage.currentTeam == PoseStorage.Team.BLUE) { // Depending on which side we're on the color changes
                    //input = drive.pose.heading.inverse().plus(Math.toRadians(90)).times(new Vector2d(-input.x, input.y)); // magic courtesy of https://github.com/acmerobotics/road-runner/issues/90#issuecomment-1722674965
                    rotationAmount = rotationAmount - Math.toRadians(90);
                } else {
                    //input = drive.pose.heading.inverse().plus(Math.toRadians(-90)).times(new Vector2d(input.x, -input.y)); // magic courtesy of
                    rotationAmount = rotationAmount + Math.toRadians(90);

                }
                input = Rotation2d.fromDouble(rotationAmount).times(new Vector2d(input.x, input.y)); // magic courtesy of https://github.com/acmerobotics/road-runner/issues/90#issuecomment-1722674965
            }
            Vector2d controllerHeading = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);

            if (drivingEnabled) {
                // check if the right stick is pushed to the edge
                // this if statement is backwards; if true, it's not pushed drive normal
                if (Math.sqrt(Math.pow(controllerHeading.x, 2.0) + Math.pow(controllerHeading.y, 2.0)) < 0.4) {
                    drive.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(
                                            input.x,
                                            input.y
                                    ),
                                    (gamepad1.left_trigger - gamepad1.right_trigger) * speed
                            )
                    );
                } else { // right stick is pushed
                    // Set the target heading for the heading controller to our desired angle

                    // Cast the angle based on the angleCast of the joystick as a heading
                    if (PoseStorage.currentTeam == PoseStorage.Team.BLUE) {
                        joystickHeadingController.targetPosition = controllerHeading.angleCast().log() + Math.toRadians(-90);
                    } else {
                        joystickHeadingController.targetPosition = controllerHeading.angleCast().log() + Math.toRadians(90);
                    }


                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput = (joystickHeadingController.update(drive.pose.heading.log())
                            * MecanumDrive.PARAMS.kV
                            * MecanumDrive.PARAMS.trackWidthTicks);
                    drive.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(
                                            input.x,
                                            input.y
                                    ),
                                    headingInput
                            )
                    );

                }
            }
        }
    }


    // TODO: probably not needed, just make a normal action
    interface input {
        boolean isPressed();
    }
    Action waitForInput (input input){
        return telemetryPacket -> input.isPressed();
    }

    boolean padRelease () {
        return !((gamepad2.right_trigger > 0.25) && previousGamepad2.right_trigger < 0.25);
    }

    Action moveBack3In() {
        return new SequentialAction(
                new InstantAction(() -> drivingEnabled = false),
                new MoveBackAction(),
                new SleepAction(0.1),
                new StopMovingAction(),
                new InstantAction(() -> drivingEnabled = true)
        );
    }

    class MoveBackAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!gamepad2.left_bumper) {
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(
                                        -1,
                                        0
                                ),
                                0
                        )
                );
            }
            return false;

        }
    }
    class StopMovingAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    0,
                                    0
                            ),
                            0
                    )
            );
            return false;
        }
    }

    boolean driveActionRunning() {
        return containsDriveAction(runningActions);
    }
    boolean containsDriveAction(List<Action> actions) {
        for (Action action : actions) {
            if (action.getClass() == MecanumDrive.FollowTrajectoryAction.class || action.getClass() == MoveBackAction.class || action.getClass() == StopMovingAction.class) {
                return true;
            }
            if (action.getClass() == SequentialAction.class) {
                SequentialAction sequentialAction = (SequentialAction) action;
                return containsDriveAction(sequentialAction.getInitialActions());
            }
        }
        return false;
    }

}
