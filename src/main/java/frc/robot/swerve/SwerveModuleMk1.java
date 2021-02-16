package frc.robot.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleMk1 implements SwerveModule {

    private final CANSparkMax mAzimuthMotor;
    private final CANSparkMax mDriveMotor;
    private final CANCoder mAzimuthCanCoder;
    private final CANEncoder mAzimuthEncoder;
    private final CANEncoder mDriveEncoder;
    @SuppressWarnings("unused")
    private final CANPIDController mDrivePID;
    private final CANPIDController mAzimuthPID;
    private final Translation2d mLocation;
    private final String mName; 
    private boolean isInverted;

    public SwerveModuleMk1(CANSparkMax azimuthMotor, CANSparkMax driveMotor, CANCoder azimuthEncoder,
            Translation2d location, String name) {
        mAzimuthCanCoder = azimuthEncoder;
        mDriveMotor = driveMotor;
        mAzimuthMotor = azimuthMotor;
        mLocation = location;
        mName = name;

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
        return SwerveState.fromDegrees(mAzimuthEncoder.getPosition(), mDriveEncoder.getVelocity());
    }

    @Override
    public void init() {
        mAzimuthMotor.getEncoder().setPositionConversionFactor(25.08);
        mAzimuthMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mAzimuthMotor.setSmartCurrentLimit(10);

        mDriveMotor.setInverted(true);
        mDriveEncoder.setVelocityConversionFactor(4.712);

        setAzimuthZero();
    }

    @Override
    public void log() {
        SmartDashboard.putNumber(mName + " Absolute Position", mAzimuthCanCoder.getAbsolutePosition());
        SmartDashboard.putNumber(mName + " Incremental Position", mAzimuthEncoder.getPosition());
        SmartDashboard.putNumber(mName + " Velocity", mDriveEncoder.getVelocity());
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

    @Override
    public void stop(){
        mAzimuthMotor.set(0);
        mDriveMotor.set(0);
    }
    
    public void setAzimuthZero() {
        //calculate position to increments
        double position = mAzimuthCanCoder.getAbsolutePosition();
        @SuppressWarnings("unused")
        CANError err = mAzimuthEncoder.setPosition(position);
    }
    
}
