/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShooterSubSystem extends SubsystemBase {

  //VictorSPX mShooterMotor;
  private CANSparkMax mShooterMotor;
  VictorSPX mFeederMotor;

  private CANPIDController mPidController;
  private CANEncoder mEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  public ShooterSubSystem() {

   // mShooterMotor = new VictorSPX(ShooterConstants.kShooterMotorPort);

    mShooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);
    mPidController = mShooterMotor.getPIDController();
    mEncoder = mShooterMotor.getEncoder();

    // PID coefficients
    kP = 0.00005; 
    kI = 0.00000025;
    kD = 0.02; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    mPidController.setP(kP);
    mPidController.setI(kI);
    mPidController.setD(kD);
    mPidController.setIZone(kIz);
    mPidController.setFF(kFF);
    mPidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);



    try {
      mFeederMotor = new VictorSPX(Constants.kFeederMotorPort );
      mFeederMotor.setNeutralMode( NeutralMode.Brake );
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error Feeder Motor Controller:  " + ex.getMessage(), true);
    }

    SmartDashboard.putString("SPark ","" + mShooterMotor.getOutputCurrent() );
  }

  public void SetShooterSpeed(double speed) {
    SmartDashboard.putString("SPark ","" + mShooterMotor.getOutputCurrent() );
    mShooterMotor.set(speed);   
  }

  public void StartFeederMotor() {
    mFeederMotor.set(ControlMode.PercentOutput, Constants.kFeederMotorSpeed) ;   
  }

  public void StopFeederMotor() {
    mFeederMotor.set(ControlMode.PercentOutput, 0) ;   
  }

  public double GetVelocity() {
    return mShooterMotor.getEncoder().getVelocity();
  }

  public void SetMotorRPM(double rpm) {
    mPidController.setReference(rpm, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
