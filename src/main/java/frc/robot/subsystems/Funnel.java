// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
  /** Creates a new Funnel. */

  private final CANSparkMax mFunnelMotor;

  private final static double funnelSpeed = 1;
  
  public Funnel() {
    mFunnelMotor = new CANSparkMax(32, MotorType.kBrushless);
  }

  public void setSpeed(double speed){
    mFunnelMotor.set(speed);
  }

  public void funnel(){
    setSpeed(funnelSpeed);
  }

  public void stop(){
    setSpeed(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
