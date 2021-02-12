package frc.robot.swerve;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public interface SwerveModule {
    //Used to run any initialization needed on the swerve module
    void init();
    //Used to log any data from the serve module
    void log();
    //Used to set the velocity and position of the swerve module
    void set(SwerveState drive);
    void stop();
    //Used to get the velocity and position of the swerve module
    SwerveState getState();
    //Used to get the location of the swerve module relative to the 
    //center of the robot
    Translation2d getModuleLocation();
}
