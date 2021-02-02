// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  VictorSPX mRollerMotor;
  Solenoid mSolenoid;

  public Intake() {

  mRollerMotor = new VictorSPX(51);
  mRollerMotor.setInverted(true);

  mSolenoid = new Solenoid(2);
}

public void setSpeed(double speed) {
  mRollerMotor.set(ControlMode.PercentOutput, speed);
}

public void lift(){
  mSolenoid.set(false);
  }


public void lower(){
  mSolenoid.set(true);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
