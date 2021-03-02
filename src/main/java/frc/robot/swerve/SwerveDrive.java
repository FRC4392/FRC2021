package frc.robot.swerve;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class SwerveDrive {
    private final SwerveModule[] mModules;
    private final int numModules;
    private final SwerveDriveKinematics mKinematics;
    private final SwerveDriveOdometry mSwerveDriveOdometry;
    private final HolonomicDriveController mDriveController;
    private final DoubleSupplier mGyroAngle;

    //move gyro out of this class?


    public SwerveDrive(DoubleSupplier gyroAngle, SwerveModule... modules){
        mGyroAngle = gyroAngle;
        numModules = modules.length;
        mModules = Arrays.copyOf(modules, numModules);

        Translation2d[] moduleLocations = new Translation2d[numModules];
        for (int i = 0; i < numModules; i++){
            moduleLocations[i] = mModules[i].getModuleLocation();
        }

        mKinematics = new SwerveDriveKinematics(moduleLocations);

        mDriveController = new HolonomicDriveController(new PIDController(1,0,0), new PIDController(1,0,0), new ProfiledPIDController(1,0,0,new TrapezoidProfile.Constraints(6.28, 3.14)));
        mSwerveDriveOdometry = new SwerveDriveOdometry(mKinematics, Rotation2d.fromDegrees(mGyroAngle.getAsDouble()));
    }

	public void stop() {
        Arrays.stream(mModules).forEach(SwerveModule::stop);
	}

    public void drive(double forward, double strafe, double azimuth, boolean fieldRelative){
        ChassisSpeeds speeds;
        if (fieldRelative){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, azimuth, Rotation2d.fromDegrees(mGyroAngle.getAsDouble()));
        } else {
            speeds = new ChassisSpeeds(forward, strafe, azimuth);
        }
        SwerveModuleState[] states = mKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, 1);
        for (int i = 0; i < numModules; i++){
            mModules[i].set(states[i]);
        }
    }

    public Pose2d updateOdometry(){
        SwerveModuleState[] states = (SwerveModuleState[]) Arrays.stream(mModules).map(SwerveModule::getState).toArray();
        return mSwerveDriveOdometry.update(Rotation2d.fromDegrees(mGyroAngle.getAsDouble()), states);
    }

    public void log(){

    }


}