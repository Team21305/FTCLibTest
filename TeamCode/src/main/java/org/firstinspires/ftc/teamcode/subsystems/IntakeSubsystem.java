package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {

    private final Servo _left;
    private final Servo _right;

    public IntakeSubsystem(Servo left, Servo right){

        _left = left;
        _right = right;
    }

    public void open() {
        _left.setPosition(0.6);
        _right.setPosition(0.6);
    }

    public void close() {
        _left.setPosition(0.4);
        _right.setPosition(0.4);
    }
}
