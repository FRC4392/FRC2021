/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  public final Intake mIntake;
  public final Funnel mFunnel;
  public final  Indexer mIndexer;

  public IntakeCommand(Intake intake, Funnel funnel, Indexer indexer) {
    mIntake = intake;
    mFunnel = funnel;
    mIndexer = indexer;
    addRequirements(mIntake, mFunnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //mIntake.lower();



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntake.intake();
    if (!mIndexer.getStartEye()){
      mFunnel.funnel();
    } else {
      mFunnel.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.stop();
    mFunnel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
