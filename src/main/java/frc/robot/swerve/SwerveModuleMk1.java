package frc.robot.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;

public class SwerveModuleMk1 implements SwerveModule {

    private final CANSparkMax mAzimuthMotor;
    private final CANSparkMax mDriveMotor;
    private final CANCoder mAzimuthCanCoder;
    private final double mPositionX;
    private final double mPositonY;

    public SwerveModuleMk1(CANSparkMax azimuthMotor, CANSparkMax driveMotor, CANCoder azimuthEncoder){
        mAzimuthCanCoder = azimuthEncoder;
        mDriveMotor = driveMotor;
        mAzimuthMotor = azimuthMotor;
    }


    @Override
    public void setSpeed() {
        mAzimuthMotor.set(0);

    }

    @Override
    public void setRotation() {
        mDriveMotor.set(0);

    }

    @Override
    public void getSpeed() {
        // TODO Auto-generated method stub

    }

    @Override
    public void getRotation() {
        mAzimuthCanCoder.getAbsolutePosition();

    }
    
}
