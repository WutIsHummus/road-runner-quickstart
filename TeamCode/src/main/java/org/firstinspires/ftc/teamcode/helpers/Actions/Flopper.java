package org.firstinspires.ftc.teamcode.helpers.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Flopper {
    private DcMotorEx motor;

    public Flopper(DcMotorEx motor) {
        this.motor = motor;
    }

    public Action moveToTarget(int targetPosition, double power) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setPower(power);
                    motor.setTargetPosition(targetPosition);
                    motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                double pos = motor.getCurrentPosition();
                packet.put("motorPosition", pos);

                if (!motor.isBusy()) {
                    motor.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        };
    }
}

