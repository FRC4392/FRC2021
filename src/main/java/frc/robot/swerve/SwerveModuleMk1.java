package frc.robot.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class SwerveModuleMk1 implements SwerveModule {

    private final CANSparkMax mAzimuthMotor;
    private final CANSparkMax mDriveMotor;
    private final CANCoder mAzimuthCanCoder;
    private final CANEncoder mAzimuthEncoder;
    private final CANEncoder mDriveEncoder;
    private final CANPIDController mDrivePID;
    private final CANPIDController mAzimuthPID;
    private final Translation2d mLocation;
    private boolean isInverted;

    public SwerveModuleMk1(CANSparkMax azimuthMotor, CANSparkMax driveMotor, CANCoder azimuthEncoder,
            Translation2d location) {
        mAzimuthCanCoder = azimuthEncoder;
        mDriveMotor = driveMotor;
        mAzimuthMotor = azimuthMotor;
        mLocation = location;

        mAzimuthEncoder = mAzimuthMotor.getEncoder();
        mDriveEncoder = mDriveMotor.getEncoder();
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
        return mDriveEncoder.getVelocity();
    }

    // Gets the rotation position of the azimuth module
    public double getRotation() {
        return mAzimuthEncoder.getPosition();
    }

    @Override
    public Translation2d getModuleLocation() {
        return mLocation;
    }

    @Override
    public SwerveState getState() {
        return SwerveState.fromDegrees(mAzimuthCanCoder.getAbsolutePosition(), mDriveEncoder.getVelocity());
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
    public void set(SwerveState drive) {
        double Angle = drive.getDegrees();
        double Velocity = drive.getVelocity();

        double azimuthPosition = mAzimuthEncoder.getPosition();
        double azimuthError = Math.IEEEremainder(Angle - azimuthPosition, 360);

        isInverted = Math.abs(azimuthError) > 90;
        if (isInverted) {
            azimuthError -= Math.copySign(180, azimuthError);
            Velocity = -Velocity;
        }
        double setpoint = azimuthError + azimuthPosition;
        mAzimuthPID.setReference(setpoint, ControlType.kPosition);
        mDriveMotor.set(Velocity);
    }

    public void stop(){
        mAzimuthMotor.set(0);
        mDriveMotor.set(0);

    }
    
}
