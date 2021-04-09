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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import org.opencv.core.Mat;

public class DriveCommand extends CommandBase {
  public final Drivetrain mDrivetrain;
  public XboxController mController;
  private Preferences preferences = Preferences.getInstance();
  private PIDController mPIDController;
  private boolean lastScan;
  
  public DriveCommand(Drivetrain Drivetrain, XboxController XboxController) {
    mDrivetrain = Drivetrain;
    mController = XboxController;
    mPIDController = new PIDController(0.01,0,0);
    //mPIDController.setPID(1,0,0);
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

      double SpeedMultiplier = preferences.getDouble("SpeedMultiplier", 0.75);




     double deadband = preferences.getDouble("ControllerDeadband", 0.03);

    if (mController.getBumper(Hand.kLeft)){
      SpeedMultiplier = .1;
      deadband = 0;
    }

    yVel = mController.getY(Hand.kLeft) * SpeedMultiplier;
    xVel = mController.getX(Hand.kLeft) * SpeedMultiplier;

      if (Math.abs(yVel) > deadband){
        yVel = (yVel-deadband)*(1/(1-deadband));
      } else {
        yVel = 0;
      }

    if (Math.abs(xVel) > deadband){
      xVel = (xVel-deadband)*(1/(1-deadband));
    } else {
      xVel = 0;
    }

//    rotVel = -mPIDController.calculate(mDrivetrain.getRotation(), 90);

      double polar = Math.sqrt(yVel*yVel)+(xVel*xVel);
      SmartDashboard.putNumber("Polar", polar);

    rotVel = mController.getX(Hand.kRight) * SpeedMultiplier;

    if (Math.abs(rotVel) > deadband){
      rotVel = (rotVel-deadband)*(1/(1-deadband));
    } else {
      rotVel = 0;
    }



    if (mController.getRawButton(7) &! lastScan){
      mDrivetrain.resetGyro();
    }
    lastScan = mController.getRawButton(7);

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
