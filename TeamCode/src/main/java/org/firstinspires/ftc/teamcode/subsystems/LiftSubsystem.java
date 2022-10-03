package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class LiftSubsystem extends SubsystemBase {

    Servo _servo;

    public LiftSubsystem(Servo serve){
        _servo = serve;
    }

    public void goUp(){
        _servo.setPosition(0.5);
    }

    public void goDown(){
        _servo.setPosition(0.0);
    }
}
