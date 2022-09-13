package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {

    private final DriveSubsystem _drive;
    private final DoubleSupplier _forward;
    private final DoubleSupplier _rotation;

    public DefaultDriveCommand(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation){

        _drive = subsystem;
        _forward = forward;
        _rotation = rotation;

        addRequirements(_drive);

    }

    @Override
    public void execute(){
        _drive.drive(_forward.getAsDouble(), _rotation.getAsDouble());

    }

}
