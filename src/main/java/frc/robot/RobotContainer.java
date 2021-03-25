// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final Drivetrain mDrivetrain = new Drivetrain();
  Limelight mLimelight = new Limelight();
  Intake mIntake = new Intake();
  Indexer mIndexer = new Indexer();
  Shooter mShooter = new Shooter();
  Funnel mFunnel = new Funnel();

  XboxController mDriverController = new XboxController(0);
  XboxController mOperatorController = new XboxController(1);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton ManualShootButton = new JoystickButton(mOperatorController,XboxController.Button.kX.value);
    JoystickButton ManualShootButton2 = new JoystickButton(mOperatorController, XboxController.Button.kA.value);
    JoystickButton ManualShootButton3 = new JoystickButton(mOperatorController, XboxController.Button.kB.value);
    JoystickButton IntakePositionButton = new JoystickButton(mOperatorController, XboxController.Button.kBumperRight.value);
    Trigger IntakeButton = new Trigger( () -> mOperatorController.getTriggerAxis(GenericHID.Hand.kLeft) > 0.01 );
    Trigger ShootButton = new Trigger( () -> mOperatorController.getTriggerAxis(GenericHID.Hand.kRight) > 0.01 );

    //Testing buttons
//    JoystickButton IntakePositionButton2 = new JoystickButton(mDriverController, XboxController.Button.kBumperRight.value);
//    IntakePositionButton2.whenPressed(mIntake::lift);
//    IntakePositionButton2.whenReleased(mIntake::lower);
    JoystickButton refreshButton = new JoystickButton(mOperatorController, XboxController.Button.kBumperLeft.value);
//
//    Trigger IntakeButton2 = new Trigger( () -> mDriverController.getTriggerAxis(GenericHID.Hand.kLeft) > 0.01 );
//    IntakeButton2.whileActiveContinuous(new IntakeCommand(mIntake, mFunnel, mIndexer));
//
//    JoystickButton DriverManualShootButton2 = new JoystickButton(mDriverController, XboxController.Button.kA.value);
//    DriverManualShootButton2.whileHeld(new ManualShootCommand2(mShooter));
//
//    Trigger DriverShootButton = new Trigger( () -> mDriverController.getTriggerAxis(GenericHID.Hand.kRight) > 0.01 );
//    DriverShootButton.whileActiveContinuous(new IndexShoot(mIndexer, mShooter));
    //End of testing buttons

    mDrivetrain.setDefaultCommand(new DriveCommand(mDrivetrain, mDriverController));
    ManualShootButton.whileHeld(new ManualShootCommand(mShooter));
    ManualShootButton2.whileHeld(new ManualShootCommand2(mShooter));
    ManualShootButton3.whileHeld( new ManualShootCommand3(mShooter));
    IntakeButton.whileActiveContinuous(new IntakeCommand(mIntake, mFunnel, mIndexer));
    IntakePositionButton.whenPressed(mIntake::lift);
    IntakePositionButton.whenReleased(mIntake::lower);
    mIndexer.setDefaultCommand(new IndexerIndexCommand(mIndexer));
    ShootButton.whileActiveContinuous(new IndexShoot(mIndexer, mShooter));

    refreshButton.whenPressed(mShooter::refreshPID);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SimpleAutoCommand(mDrivetrain);
  }
}
