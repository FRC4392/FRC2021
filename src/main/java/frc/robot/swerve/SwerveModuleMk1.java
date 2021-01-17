package frc.robot.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;

public class SwerveModuleMk1 implements SwerveModule {

    private final CANSparkMax mAzimuthMotor;
    private final CANSparkMax mDriveMotor;
    private final CANCoder mAzimuthCanCoder;
    private final double mPositionX;
    private final double mPositionY;

    public SwerveModuleMk1(CANSparkMax azimuthMotor, CANSparkMax driveMotor, CANCoder azimuthEncoder, double X, double Y){
        mAzimuthCanCoder = azimuthEncoder;
        mDriveMotor = driveMotor;
        mAzimuthMotor = azimuthMotor;
        mPositionX = X;
        mPositionY = Y;
    }


    @Override
    public void setSpeed(double speed) {
        mAzimuthMotor.set(0);

    }

    @Override
    public void setRotation(double rotation) {
        mDriveMotor.set(0);

    }

    @Override
    public double getSpeed() {
        // TODO Auto-generated method stub

    }

    @Override
    public double getRotation() {
        mAzimuthCanCoder.getAbsolutePosition();

    }

    @Override
    public double getPositionX() {
        return mPositionX;
    }

    @Override
    public double getPositionY() {
        return mPositionY;
    }
    
}
