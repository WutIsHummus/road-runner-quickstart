package org.firstinspires.ftc.teamcode.helpers.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class Box {
    private Servo box;

    public Box(Servo box) {
        this.box = box;
    }
    public Action moveTo(final double position) {
        return packet -> {
            box.setPosition(position);
            return false;
        };
    }
}

