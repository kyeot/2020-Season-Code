/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NeoLiftSubSystem extends SubsystemBase {
  /**
   * Creates a new NeoLiftSubSystem.
   */
  CANSparkMax mLiftMotor;
  private DutyCycleEncoder mEncoder;
  public NeoLiftSubSystem() {
    try {
      mLiftMotor = new CANSparkMax(Constants.kLiftMotorPort, null);
      //mLiftMotor.setNeutralMode( NeutralMode.Brake );
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error Lift Motor Controller:  " + ex.getMessage(), true);
    }

    try {
      mEncoder = new DutyCycleEncoder(4);
      mEncoder.setDistancePerRotation(1);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error Lift Lift Encoder:  " + ex.getMessage(), true);
    }




  }

  public double GetEncoderDistance() {
    return mEncoder.getDistance();
  }

  public void ResetEncoder() {
    mEncoder.reset();
  }

  public void RaiseLift() {
    mLiftMotor.set( -Constants.kLiftMotorSpeed );
  }

  public void LowerLift() {
    mLiftMotor.set( Constants.kLiftMotorSpeed );
  }

  public void StopLift() {
    mLiftMotor.set(0);
  }

  public void SetLiftSpeed(double speed) {
    mLiftMotor.set(speed); 
  }

  /*
  public boolean isSwitchSet() {
    return counter.get() > 0;
  }

  public void ResetSwitch()
  {
    counter.reset();
  }
  */
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
