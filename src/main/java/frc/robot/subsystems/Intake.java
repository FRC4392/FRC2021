// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax mRollerMotor;
  private final Solenoid mSolenoid;

  private static final double intakeSpeed = 1;
  private static final boolean liftPosition = false;
  private static final boolean lowerPosition = true;

  public Intake() {

  mRollerMotor = new CANSparkMax(51, CANSparkMaxLowLevel.MotorType.kBrushless);
  mRollerMotor.setSmartCurrentLimit(20, 40, 11000);


  mSolenoid = new Solenoid(1);
}

public void setSpeed(double speed) {
  mRollerMotor.set(speed);
}

public void intake(){
    setSpeed(intakeSpeed);
}

public void stop(){
    mRollerMotor.set(0);
}

public void lift(){
  mSolenoid.set(liftPosition);
  }

public void lower(){
  mSolenoid.set(lowerPosition);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
