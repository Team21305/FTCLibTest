package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class DriveSubsystem extends SubsystemBase{

    private final DifferentialDrive _drive;
    private final Encoder _leftEnc;
    @androidx.annotation.NonNull
    private final MotorEx _leftMotor;
    private final Encoder _rightEnc;

    private final double WHEEL_DIAMETER;

    public DriveSubsystem(MotorEx leftMotor, MotorEx rightMotor, final double diameter) {
        _leftEnc = leftMotor.encoder;
        _leftMotor = leftMotor;
        _rightEnc = rightMotor.encoder;

        WHEEL_DIAMETER = diameter;

        _drive = new DifferentialDrive(leftMotor, rightMotor);
    }

    @Override
    public void periodic() {
        super.periodic();
        updateTelemetry();
    }

    public void drive(double fwd, double rot){
        _drive.arcadeDrive(fwd, rot);
    }

    public double getLeftEncoderVal(){
        return _leftEnc.getPosition();
    }

    public double getRightEncoderVal(){
        return _rightEnc.getPosition();
    }

    public double getLeftEncoderDistance(){
        return _leftEnc.getRevolutions() * WHEEL_DIAMETER * Math.PI;
    }

    public double getRightEncoderDistance(){
        return _rightEnc.getRevolutions() * WHEEL_DIAMETER * Math.PI;
    }

    public void resetEncoders(){
        _leftEnc.reset();
        _rightEnc.reset();
    }

    public double getAverageDistance(){
        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
    }

    private void updateTelemetry(){
        double pow = _leftMotor.motor.getPower();
        telemetry.addData("Left Power", pow);

        telemetry.update();
    }
}
