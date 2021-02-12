// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveModuleMk1;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax mDriveMotor1 = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax mDriveMotor2 = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax mDriveMotor3 = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax mDriveMotor4 = new CANSparkMax(14, MotorType.kBrushless);

  private final CANSparkMax mAzimuth1 = new CANSparkMax(21, MotorType.kBrushless);
  private final CANSparkMax mAzimuth2 = new CANSparkMax(22, MotorType.kBrushless);
  private final CANSparkMax mAzimuth3 = new CANSparkMax(23, MotorType.kBrushless);
  private final CANSparkMax mAzimuth4 = new CANSparkMax(24, MotorType.kBrushless);

  private final CANCoder mCanCoder1 = new CANCoder(11);
  private final CANCoder mCanCoder2 = new CANCoder(12);
  private final CANCoder mCanCoder3 = new CANCoder(13);
  private final CANCoder mCanCoder4 = new CANCoder(14);

  private final PigeonIMU pidgey = new PigeonIMU(10);

  private final double mTrackWidth = 30.0;
  private final double mWheelBase = 30.0;

  private final SwerveModuleMk1 Module1 = new SwerveModuleMk1(mAzimuth1, mDriveMotor1, mCanCoder1, new Translation2d(), "");
  private final SwerveModuleMk1 Module2 = new SwerveModuleMk1(mAzimuth2, mDriveMotor2, mCanCoder2, new Translation2d(), "");
  private final SwerveModuleMk1 Module3 = new SwerveModuleMk1(mAzimuth3, mDriveMotor3, mCanCoder3, new Translation2d(), "");
  private final SwerveModuleMk1 Module4 = new SwerveModuleMk1(mAzimuth4, mDriveMotor4, mCanCoder4, new Translation2d(), "");

  SwerveDrive swerveDrive = new SwerveDrive(Module1, Module2, Module3, Module4);


  private final SwerveDrive mSwerve;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    mSwerve = new SwerveDrive(Module1, Module2, Module3, Module4);

  }

  public void drive(double forward, double strafe, double azimuth){
    swerveDrive.drive(forward, strafe, azimuth);
  }

  public void stop(){
    swerveDrive.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
