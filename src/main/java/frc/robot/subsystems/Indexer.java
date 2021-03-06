// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final CANSparkMax mIndexerMotor;
  private final CANifier mCanifier;
  private final static double indexSpeed = 0.35;
  private final static double feedSpeed = .8;

  public Indexer() {
    mIndexerMotor = new CANSparkMax(41, MotorType.kBrushless);

    mCanifier = new CANifier(42);

    mIndexerMotor.burnFlash();

    mIndexerMotor.setInverted(true);

    mIndexerMotor.setSmartCurrentLimit(30);
  }

  public void setSpeed(double speed) {
    mIndexerMotor.set(speed);
  }

  public void index(){
    setSpeed(indexSpeed);
  }

  public void feed(){
    setSpeed(feedSpeed);
  }

  public void stop(){
    setSpeed(0);
  }

  public boolean getStartEye(){
    return !mCanifier.getGeneralInput(GeneralPin.SPI_MISO_PWM2P);
  }

  public boolean getEndEye() {
    return !mCanifier.getGeneralInput(GeneralPin.SPI_MOSI_PWM1P);
  }

  public void log(){
    SmartDashboard.putBoolean("endEyeState", getEndEye());
    SmartDashboard.putBoolean("startEyeState", getStartEye());
    SmartDashboard.putNumber("IndexerSpeed", mIndexerMotor.getAppliedOutput());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
