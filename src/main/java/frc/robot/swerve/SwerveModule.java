package frc.robot.swerve;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public interface SwerveModule {
    /**
     * Run any initialization needed on the swerve module
     */
    void init();
    /**
     * Log necassary data to driverstation and file
     */
    void log();
    /**
     * Set the state the swerve drive should attempt to be in
     */
    void set(SwerveState drive);
    /**
     * Stop all motion on the swerve drive.
     */
    void stop();
    /**
     * Return the current state of the swerve module
     */
    SwerveState getState();
    /**
     * Get the location of the swerve module relative to the 
     * center of the robot
     */
    Translation2d getModuleLocation();
}
