package org.firstinspires.ftc.teamcode.helpers.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Lift {
    private DcMotorEx left;
    private DcMotorEx right;

    public Lift(DcMotorEx left, DcMotorEx right) {
        this.left = left;
        this.right = right;
    }

    public Action moveToTarget(int targetPosition, double power) {
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    left.setPower(power);
                    left.setTargetPosition(targetPosition);
                    left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    right.setPower(power);
                    right.setTargetPosition(targetPosition);
                    right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                double posR = right.getCurrentPosition();
                double posL = left.getCurrentPosition();
                packet.put("LeftPosition", posL);
                packet.put("RightPosition", posR);

                if (!right.isBusy() || !left.isBusy()) {
                    right.setPower(0);
                    left.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        };
    }
}

