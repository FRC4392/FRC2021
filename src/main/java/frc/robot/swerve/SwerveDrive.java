package frc.robot.swerve;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Timer;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

public class SwerveDrive {
    private final SwerveModule[] mModules;
    private final int numModules;
    private final SwerveDriveKinematics mKinematics;
    private final SwerveDriveOdometry mSwerveDriveOdometry;
    private final HolonomicDriveController mDriveController;
    private final DoubleSupplier mGyroAngle;
    private final Trajectory trajectory;

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

        mDriveController = new HolonomicDriveController(new PIDController(0.1,0,0), new PIDController(0.1,0,0), new ProfiledPIDController(1,.1,.1,new TrapezoidProfile.Constraints(6.28, 3.14)));
        mSwerveDriveOdometry = new SwerveDriveOdometry(mKinematics, Rotation2d.fromDegrees(mGyroAngle.getAsDouble()));

        Arrays.stream(mModules).forEach(SwerveModule::init);

        TrajectoryConfig config = new TrajectoryConfig(.5, 1);
        config.setKinematics(mKinematics);
        //config.setReversed(true);
        config.addConstraint(new SwerveDriveKinematicsConstraint(mKinematics, .5));
        
        trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(), new ArrayList<Translation2d>(), new Pose2d(2.0, 0.0, new Rotation2d()), config);
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

    public Pose2d getPosition(){
        return mSwerveDriveOdometry.getPoseMeters();
    }

    public Pose2d updateOdometry(){
        SwerveModuleState[] states = new SwerveModuleState[numModules];
        for (int i = 0; i < numModules; i++) {
            states[i] = mModules[i].getState();
        }
        return mSwerveDriveOdometry.update(Rotation2d.fromDegrees(mGyroAngle.getAsDouble()), states);
    }

    public void followPath(double startTime){
        Trajectory.State goal = trajectory.sample(Timer.getFPGATimestamp() - startTime);

        ChassisSpeeds speeds = mDriveController.calculate(getPosition(), goal, new Rotation2d());

        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond, false);
        SmartDashboard.putNumber("goalx", goal.poseMeters.getX());
        SmartDashboard.putNumber("xerror", goal.poseMeters.getX() - getPosition().getX());
        SmartDashboard.putNumber("goaly", goal.poseMeters.getY());
        SmartDashboard.putNumber("yerror", goal.poseMeters.getY() - getPosition().getY());
        SmartDashboard.putNumber("goalrot", goal.poseMeters.getRotation().getDegrees());
    }

    public void log(){
        Arrays.stream(mModules).forEach(SwerveModule::log);
        Pose2d pose = getPosition();
        SmartDashboard.putNumber("SwerveXLocation", pose.getX());
        SmartDashboard.putNumber("SwerveYLocation", pose.getY());
        SmartDashboard.putNumber("SwerveRotation", pose.getRotation().getDegrees());
    }


}