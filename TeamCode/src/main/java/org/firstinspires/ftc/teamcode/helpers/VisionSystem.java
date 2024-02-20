package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VisionSystem {

    private HuskyLens huskyLens;
    private Telemetry telemetry;

    final double FOCAL_LENGTH = 6.8;
    final int FOV = 65;
    final double REAL_OBJECT_WIDTH = 80.55;
    final double CAM_HEIGHT = 11.0236;
    final int CAM_ANGLE = 45;

    public VisionSystem(HuskyLens huskyLens, Telemetry telemetry) {
        this.huskyLens = huskyLens;
        this.telemetry = telemetry;
        initializeHuskyLens();
    }

    private void initializeHuskyLens() {
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "HuskyLens Initialized");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
    }

    public double calculateDistance(double widthInFrame) {
        return ((REAL_OBJECT_WIDTH * FOCAL_LENGTH) / widthInFrame) * 3.937;
    }

    public double calculateHorizontalRotationAngle(int objectPosX, int imageWidth) {
        int imageCenterX = 275; // Assuming 275 is the center for your specific setup
        double offsetFromCenter = objectPosX - imageCenterX;
        return ((offsetFromCenter / (double) imageWidth) * FOV) + 4;
    }

    public HuskyLens.Block[] getBlocks() {
        return huskyLens.blocks();
    }
}
