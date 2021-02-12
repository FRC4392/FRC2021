package frc.robot.swerve;

import java.util.Arrays;

public class SwerveDrive {
    private final SwerveModule[] mModules;
    private final int numModules;

    public SwerveDrive(SwerveModule... modules){
        numModules = modules.length;
        mModules = Arrays.copyOf(modules, numModules);
    }

	public void stop() {
        for (SwerveModule swerveModule : mModules) {
            swerveModule.stop();
        }
	}

    public void drive(double forward, double strafe, double azimuth){
        
    }


}
