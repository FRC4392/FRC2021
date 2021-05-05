package frc.robot.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.core.Mat;

public class SwerveModuleMk1 implements SwerveModule {

    private final CANSparkMax mAzimuthMotor;
    private final CANSparkMax mDriveMotor;
    private final CANCoder mAzimuthCanCoder;
    private final CANEncoder mAzimuthEncoder;
    private final CANEncoder mDriveEncoder;
    private final CANPIDController mDrivePID;
    private final CANPIDController mAzimuthPID;
    private final Translation2d mLocation;
    private final String mName; 
    private boolean isInverted;
    private double setpoint;

    //need to update the speed to m/s

    public SwerveModuleMk1(CANSparkMax azimuthMotor, CANSparkMax driveMotor, CANCoder azimuthEncoder,
            Translation2d location, String name) {
        mAzimuthCanCoder = azimuthEncoder;
        mDriveMotor = driveMotor;
        mAzimuthMotor = azimuthMotor;
        mLocation = location;
        mName = name;

        mAzimuthEncoder = mAzimuthMotor.getEncoder();
        mDriveEncoder = mDriveMotor.getEncoder();
        mDriveEncoder.setPositionConversionFactor(.004356);
        //mDriveEncoder.setVelocityConversionFactor((0.0241/42.0));
        mDriveEncoder.setVelocityConversionFactor((0.029/42.0));
        mDriveEncoder.setPosition(0);
        mDrivePID = mDriveMotor.getPIDController();
        mDrivePID.setFF(0.30);
        mDrivePID.setP(0.5);
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

    //@Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getRotation()));
    }

    @Override
    public void init() {
        mAzimuthMotor.getEncoder().setPositionConversionFactor(25.08);
        mAzimuthMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mAzimuthMotor.setSmartCurrentLimit(10);

        mAzimuthPID.setP(0.05);

        mDriveMotor.setInverted(true);
        mDriveMotor.setClosedLoopRampRate(0);

        setAzimuthZero();
    }

    @Override
    public void log() {
        SmartDashboard.putNumber(mName + " Absolute Position", mAzimuthCanCoder.getAbsolutePosition());
        SmartDashboard.putNumber(mName + " Incremental Position", mAzimuthEncoder.getPosition());
        SmartDashboard.putNumber(mName + " Velocity", mDriveEncoder.getVelocity());
        SmartDashboard.putNumber(mName + "Drive Encoder Position", mDriveEncoder.getPosition());
        SmartDashboard.putNumber(mName + " Rotation Setpoint", setpoint);
    }

    @Override
    public void set(SwerveModuleState drive) {

        if (Math.abs(mAzimuthEncoder.getPosition() - mAzimuthCanCoder.getAbsolutePosition()) > 5){
            setAzimuthZero();
        }

        double Angle = drive.angle.getDegrees();
        SmartDashboard.putNumber(mName + " Given Setpoint", Angle);
        double Velocity = drive.speedMetersPerSecond;

        double azimuthPosition = mAzimuthEncoder.getPosition();
        double azimuthError = Math.IEEEremainder(Angle - azimuthPosition, 360);
        SmartDashboard.putNumber(mName + " Azimuth Error", azimuthError);

        isInverted = Math.abs(azimuthError) > 90;
        if (isInverted) {
            azimuthError -= Math.copySign(180, azimuthError);
            Velocity = -Velocity;
        }
        setpoint = azimuthError + azimuthPosition;
        SmartDashboard.putNumber(mName + " Azimuth CalcSetPoint", setpoint);
        mAzimuthPID.setReference(setpoint, ControlType.kPosition);
        mDriveMotor.set(Velocity);
    }

    @Override
    public void setClosedLoop(SwerveModuleState drive){
        if (Math.abs(mAzimuthEncoder.getPosition() - mAzimuthCanCoder.getAbsolutePosition()) > 1){
            //setAzimuthZero();
        }

        double Angle = drive.angle.getDegrees();
        SmartDashboard.putNumber(mName + " Given Setpoint", Angle);
        double Velocity = drive.speedMetersPerSecond;

        double azimuthPosition = mAzimuthEncoder.getPosition();
        double azimuthError = Math.IEEEremainder(Angle - azimuthPosition, 360);
        SmartDashboard.putNumber(mName + " Azimuth Error", azimuthError);

        isInverted = Math.abs(azimuthError) > 90;
        if (isInverted) {
            azimuthError -= Math.copySign(180, azimuthError);
            Velocity = -Velocity;
        }
        setpoint = azimuthError + azimuthPosition;
        SmartDashboard.putNumber(mName + " Azimuth CalcSetPoint", setpoint);
        mAzimuthPID.setReference(setpoint, ControlType.kPosition);
        SmartDashboard.putNumber(mName + " Wheel Setpoint", Velocity);
        mDrivePID.setReference(Velocity, ControlType.kVelocity);
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
