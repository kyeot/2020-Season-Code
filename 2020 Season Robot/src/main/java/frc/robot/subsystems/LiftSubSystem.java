/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class LiftSubSystem extends SubsystemBase {

  VictorSPX mLiftMotor;
  //DigitalInput limitSwitch = new DigitalInput(4);
  //Counter counter = new Counter(limitSwitch);

  private DutyCycleEncoder mEncoder;

  public LiftSubSystem() {

    

    try {
      mLiftMotor = new VictorSPX(Constants.kLiftMotorPort);
      mLiftMotor.setNeutralMode( NeutralMode.Brake );
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
    mLiftMotor.set(ControlMode.PercentOutput, -Constants.kLiftMotorSpeed );
  }

  public void LowerLift() {
    mLiftMotor.set(ControlMode.PercentOutput, Constants.kLiftMotorSpeed );
  }

  public void StopLift() {
    mLiftMotor.set(ControlMode.PercentOutput, 0 );
  }

  public void SetLiftSpeed(double speed) {
    mLiftMotor.set(ControlMode.PercentOutput, speed);
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
