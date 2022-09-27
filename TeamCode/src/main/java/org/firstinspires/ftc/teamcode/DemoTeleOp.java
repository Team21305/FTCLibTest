package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Sample TeleOp")
//@Disabled
public class DemoTeleOp extends CommandOpMode {

    private MotorEx _left, _right, _leftRear, _rightRear;
    private BNO055IMU _imu = null;
    private DriveSubsystem _drive;
    private GamepadEx _driverOp;
    private DefaultDriveCommand _defaultDriveCommand;
    private Button _upButton, _downButton;

    @Override
    public void initialize() {

        // Create Motors
        _left = new MotorEx(hardwareMap, "driveLeftFront");
        _right = new MotorEx(hardwareMap, "driveRightFront");
        _leftRear = new MotorEx(hardwareMap, "driveLeftRear");
        _rightRear = new MotorEx(hardwareMap, "driveRightRear");

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        _imu = hardwareMap.get(BNO055IMU.class, "imu");
        _imu.initialize(parameters);

        // Create DriveSubsystem
        _drive = new DriveSubsystem(_left, _right, _leftRear, _rightRear, _imu, telemetry, 100.0);

        // Create GamePad
        _driverOp = new GamepadEx(gamepad1);

        // Create DefaultDrive command
        _defaultDriveCommand = new DefaultDriveCommand(_drive, ()->_driverOp.getLeftY(), ()->_driverOp.getRightX(), true);

        // Register the drive subsystem with the scheduler
        register(_drive);

        // make DefaultDrive the default command for the drive subsystem
        _drive.setDefaultCommand(_defaultDriveCommand);

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
    }

}
