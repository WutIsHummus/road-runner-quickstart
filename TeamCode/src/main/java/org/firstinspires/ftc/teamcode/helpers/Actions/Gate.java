package org.firstinspires.ftc.teamcode.helpers.Actions;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {
    private Servo gate;

    public Gate(Servo box) {
        this.gate = box;
    }
    public Action moveTo(final double position) {
        return packet -> {
            gate.setPosition(position);
            return false;
        };
    }
}
