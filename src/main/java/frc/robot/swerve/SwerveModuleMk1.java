package frc.robot.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class SwerveModuleMk1 implements SwerveModule {

    private final CANSparkMax mAzimuthMotor;
    private final CANSparkMax mDriveMotor;
    private final CANCoder mAzimuthCanCoder;
    private final CANEncoder mAzimuthEncoder;
    private final CANEncoder mDrivEncoder;
    private final CANPIDController mDrivePID;
    private final CANPIDController mAzimuthPID;
    private final Translation2d mLocation;

    public SwerveModuleMk1(CANSparkMax azimuthMotor, CANSparkMax driveMotor, CANCoder azimuthEncoder,
            Translation2d location) {
        mAzimuthCanCoder = azimuthEncoder;
        mDriveMotor = driveMotor;
        mAzimuthMotor = azimuthMotor;
        mLocation = location;

        mAzimuthEncoder = mAzimuthMotor.getEncoder();
        mDrivEncoder = mDriveMotor.getEncoder();
        mDrivePID = mDriveMotor.getPIDController();
        mAzimuthPID = mAzimuthMotor.getPIDController();
    }

    // Sets the drive motor speed in open loop mode
    public void setSpeed(double speed) {
        mDriveMotor.set(speed);
    }

    // Sets the rotation speed of the azimuth motor in open loop mode
    public void setRotation(double rotation) {
        mAzimuthMotor.set(rotation);
    }

    // Gets the speed of the drive motor
    public double getSpeed() {
        return mDrivEncoder.getVelocity();
    }

    // Gets the rotation position of the azimuth module
    public double getRotation() {
        return mAzimuthEncoder.getPosition();
    }

    @Override
    public Translation2d getLocation() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Vector2d getState() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void init() {
        // TODO Auto-generated method stub

    }

    @Override
    public void log() {
        // TODO Auto-generated method stub

    }

    @Override
    public void set(Vector2d drive) {
        // TODO Auto-generated method stub

    }


    
}
