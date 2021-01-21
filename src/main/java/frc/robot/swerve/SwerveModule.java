package frc.robot.swerve;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public interface SwerveModule {
    void setSpeed(double speed);
    void setRotation(double rotation);
    double getSpeed();
    double getRotation();
    void set(Vector2d drive);
    Vector2d getState();
    void innit();
    void log();
    void set(Translation2d drive);
    Translation2d getLocation();
}
