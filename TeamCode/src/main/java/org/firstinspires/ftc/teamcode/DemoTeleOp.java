package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Sample TeleOp")
@Disabled
public class DemoTeleOp extends CommandOpMode {

    private MotorEx _left, _right;
    private DriveSubsystem _drive;
    private GamepadEx _driverOp;
    private DefaultDriveCommand _defaultDriveCommand;
    private Button _upButton, _downButton;

    @Override
    public void initialize() {

        // Create Motors
        _left = new MotorEx(hardwareMap, "drive_left");
        _right = new MotorEx(hardwareMap, "drive_right");

        // Create DriveSubsystem
        _drive = new DriveSubsystem(_left, _right, 100.0);

        // Create Gamepad
        _driverOp = new GamepadEx(gamepad1);

        // Create DefaultDrive command
        _defaultDriveCommand = new DefaultDriveCommand(_drive, ()->_driverOp.getLeftY(), ()->_driverOp.getLeftX());

        // Register the drive subsystem with the scheduler
        register(_drive);

        // make DefaultDrive the default command for the drive subsystem
        _drive.setDefaultCommand(_defaultDriveCommand);

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
    }

}
