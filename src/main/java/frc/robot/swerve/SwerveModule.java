package frc.robot.swerve;

public interface SwerveModule {
    void setSpeed(double speed);
    void setRotation(double rotation);
    double getSpeed();
    double getRotation();
    double getPositionX();
    double getPositionY();
}
