package frc.robot.swerve;

import java.util.Arrays;

public class SwerveDrive {
    private final SwerveModule[] mModules;
    private final int numModules;

    public SwerveDrive(SwerveModule... modules){
        numModules = modules.length;
        mModules = Arrays.copyOf(modules, numModules);
    }
}
