/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import org.opencv.core.Mat;

public class DriveCommand extends CommandBase {
  public final Drivetrain mDrivetrain;
  public XboxController mController;
  private Preferences preferences = Preferences.getInstance();
  
  public DriveCommand(Drivetrain Drivetrain, XboxController XboxController) {
    mDrivetrain = Drivetrain;
    mController = XboxController;

    addRequirements(mDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVel = 0;
    double yVel = 0;
    double rotVel = 0;
    if (Math.abs(mController.getY(Hand.kLeft)) > preferences.getDouble("ControllerDeadband", 0.03)){
      yVel = mController.getY(Hand.kLeft);
    }
    if (Math.abs(mController.getX(Hand.kLeft)) > preferences.getDouble("ControllerDeadband", 0.03)){
      xVel = mController.getX(Hand.kLeft);
    }
    if (Math.abs(mController.getX(Hand.kRight)) > preferences.getDouble("ControllerDeadband", 0.03)){
      rotVel = mController.getX(Hand.kRight);
    }

    boolean fieldRelative = !mController.getBumper(Hand.kRight);
    mDrivetrain.drive(yVel, xVel, rotVel, fieldRelative);

    SmartDashboard.putNumber("X Vel", mController.getX(Hand.kLeft));
    SmartDashboard.putNumber("Y Vel", mController.getY(Hand.kLeft));
    SmartDashboard.putNumber("Rot Vel", mController.getX(Hand.kRight));
    SmartDashboard.putNumber("Deadband", preferences.getDouble("ControllerDeadband", 0.03));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
