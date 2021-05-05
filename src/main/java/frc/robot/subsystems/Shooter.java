// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  Preferences mRobotPreferences = Preferences.getInstance();
  CANSparkMax mWheelMotor;
  CANSparkMax mWheelMotor2;
  CANPIDController mPidController;
  CANEncoder mEncoder;
  Solenoid mSolenoid;
  private double setpoint;

  private Double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  public enum HoodPosition{
    Open(true),
    Closed(false);

    private final boolean solenoidState;

    HoodPosition(boolean state){
      solenoidState = state;
    }

    private boolean getSolenoidState() {
      return solenoidState;
    }
  }


  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    mWheelMotor = new CANSparkMax(61, MotorType.kBrushless);
    mWheelMotor2 = new CANSparkMax(62, MotorType.kBrushless);

    mWheelMotor.setInverted(true);
    mWheelMotor2.setInverted(true);

    mPidController = mWheelMotor.getPIDController();
    mEncoder = mWheelMotor.getEncoder();

    mSolenoid = new Solenoid(0);

    refreshPID();


    mWheelMotor2.follow(mWheelMotor);

    mWheelMotor.setOpenLoopRampRate(0.0);
    mWheelMotor.setClosedLoopRampRate(0.1);

    mWheelMotor.burnFlash();
    mWheelMotor2.burnFlash();

  }
    public void setVelocity(double velocity) {
      mWheelMotor.set(velocity);
      setpoint = velocity;
    }

    public void setPIDVelocity(double velocity) {
      mPidController.setReference(velocity, ControlType.kVelocity);
      setpoint = velocity;
    }

    public void setHood(HoodPosition position){
      mSolenoid.set(position.getSolenoidState());
    }

    public boolean isAtSpeed(){
      return (Math.abs(setpoint - mEncoder.getVelocity()) < mRobotPreferences.getDouble("ShooterError", 100) && setpoint > 100);
    }

    public void refreshPID(){
      kP = mRobotPreferences.getDouble("ShooterKP", 0.005);
      kI = mRobotPreferences.getDouble("ShooterKI", 0.00005);
      kD = mRobotPreferences.getDouble("ShooterKD", 0.0);
      kIz = mRobotPreferences.getDouble("ShooterKIz", 0);
      kFF = mRobotPreferences.getDouble("ShooterKFF", 0.0);
      kMaxOutput = mRobotPreferences.getDouble("ShooterKMaxOutput", 1.0);
      kMinOutput = mRobotPreferences.getDouble("ShooterKMinOutput", 0);
      maxRPM = 5700.0;

      mPidController.setP(kP);
      mPidController.setI(kI);
      mPidController.setD(kD);
      mPidController.setIZone(kIz);
      mPidController.setFF(kFF);
      mPidController.setOutputRange(kMinOutput, kMaxOutput);
      mPidController.setSmartMotionMaxVelocity(5700, 0);
      mPidController.setSmartMotionMaxAccel(5700, 0);
      mPidController.setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kSCurve, 0);
    }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShooterSpeed", mEncoder.getVelocity());
  }
}
