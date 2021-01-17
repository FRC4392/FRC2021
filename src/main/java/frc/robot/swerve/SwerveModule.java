package frc.robot.swerve;

public interface SwerveModule {
    void setSpeed();
    void setRotation();
    double getSpeed();
    double getRotation();
    double getPositionX();
    double getPositionY();
}
