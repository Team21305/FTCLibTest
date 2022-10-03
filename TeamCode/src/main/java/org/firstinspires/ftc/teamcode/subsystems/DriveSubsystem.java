package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveSubsystem extends SubsystemBase{

    private final DifferentialDrive _drive;
    private final Encoder _leftEnc;
    @androidx.annotation.NonNull
    private final MotorEx _leftMotor;
    private final MotorEx _rightMotor;
    private final Encoder _rightEnc;

    private final Telemetry _telem;
    private final double WHEEL_DIAMETER;
    private final BNO055IMU _imu;

    public DriveSubsystem(MotorEx leftMotor, MotorEx rightMotor, MotorEx leftRearMotor, MotorEx rightRearMotor, BNO055IMU imu, Telemetry telem, final double diameter) {
        _leftEnc = leftMotor.encoder;
        _leftMotor = leftMotor;
        _rightMotor = rightMotor;
        _rightEnc = rightMotor.encoder;
        _telem = telem;

        WHEEL_DIAMETER = diameter;

        _imu = imu;

        _drive = new DifferentialDrive(new MotorGroup(leftMotor, leftRearMotor), new MotorGroup(rightMotor, rightRearMotor));
    }

    @Override
    public void periodic() {
        super.periodic();
        updateTelemetry();
        updateOdometry();
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

    private void updateOdometry(){}

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles   = _imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    private void updateTelemetry(){
        double pow = _leftMotor.motor.getPower();
        double powR = _rightMotor.motor.getPower();
        _telem.addData("Left Power", pow);
        _telem.addData("Right Power", powR);

        _telem.addData(">", "Robot Heading = %4.0f", getRawHeading());

        _telem.update();
    }
}
