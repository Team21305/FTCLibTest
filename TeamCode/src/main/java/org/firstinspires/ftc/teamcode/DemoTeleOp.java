package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;


@TeleOp(name = "Sample TeleOp")
//@Disabled
public class DemoTeleOp extends CommandOpMode {

    private MotorEx _left, _right, _leftRear, _rightRear;
    private BNO055IMU _imu = null;
    private DriveSubsystem _drive;
    private GamepadEx _driverOp;
    private DefaultDriveCommand _defaultDriveCommand;
    private Button _upButton, _downButton;
    private Servo _liftServo1;
    private LiftSubsystem _lift;

    @Override
    public void initialize() {

        // Create Motors
        _left = new MotorEx(hardwareMap, "driveLeftFront", Motor.GoBILDA.RPM_312);
        _right = new MotorEx(hardwareMap, "driveRightFront", Motor.GoBILDA.RPM_312);
        _leftRear = new MotorEx(hardwareMap, "driveLeftRear", Motor.GoBILDA.RPM_312);
        _rightRear = new MotorEx(hardwareMap, "driveRightRear", Motor.GoBILDA.RPM_312);

        _liftServo1 = hardwareMap.get(Servo.class, "servo1");

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        _imu = hardwareMap.get(BNO055IMU.class, "imu");
        _imu.initialize(parameters);

        // Create DriveSubsystem
        _drive = new DriveSubsystem(_left, _right, _leftRear, _rightRear, _imu, telemetry, 100.0);
        _lift = new LiftSubsystem(_liftServo1);

        // Create GamePad
        _driverOp = new GamepadEx(gamepad1);

        // Create DefaultDrive command
        _defaultDriveCommand = new DefaultDriveCommand(_drive, ()->_driverOp.getLeftY(), ()->_driverOp.getRightX(), true);

        // Register the drive subsystem with the scheduler
        register(_drive);
        register(_lift);

        // make DefaultDrive the default command for the drive subsystem
        _drive.setDefaultCommand(_defaultDriveCommand);

        _driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(()->_lift.goUp()));
        _driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(()->_lift.goDown()));

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
    }

}
