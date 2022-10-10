package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {

    private final DriveSubsystem _drive;
    private final DoubleSupplier _forward;
    private final DoubleSupplier _rotation;
    private final boolean _squareInputs;

    public DefaultDriveCommand(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation, boolean squareInputs){

        _drive = subsystem;
        _forward = forward;
        _rotation = rotation;
        _squareInputs = squareInputs;

        addRequirements(_drive);
    }

    @Override
    public void execute(){

        double y = _forward.getAsDouble();
        double r = _rotation.getAsDouble() * 0.55;

        _drive.drive(y, r, _squareInputs);

    }

}
