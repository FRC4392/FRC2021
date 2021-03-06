package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Indexer;

public class IndexerIndexCommand extends CommandBase {
    private final Indexer mIndexer;

    public IndexerIndexCommand(Indexer indexer){
        mIndexer = indexer;
        addRequirements(mIndexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (mIndexer.getStartEye() && !mIndexer.getEndEye()){
            mIndexer.index();
        } else {
            mIndexer.stop();
        }

        mIndexer.log();
    }

    @Override
    public void end(boolean interrupted) {
        mIndexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
