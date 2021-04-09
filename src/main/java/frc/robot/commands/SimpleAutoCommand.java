// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;



public class SimpleAutoCommand extends CommandBase {
  private final Drivetrain mDrivetrain;
  private double inittime;
  /** Creates a new SimpleAutoCommand. */
  public SimpleAutoCommand(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    mDrivetrain = drivetrain;

    addRequirements(mDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inittime = Timer.getFPGATimestamp();
    mDrivetrain.resetGyro();
    mDrivetrain.setStartPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrivetrain.followPath(inittime);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
