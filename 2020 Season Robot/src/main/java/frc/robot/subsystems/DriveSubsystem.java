/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class DriveSubsystem extends SubsystemBase {

  
	TalonSRX leftMotor;
	TalonSRX rightMotor;

  public DriveSubsystem() {

		leftMotor = new TalonSRX(3); //Constants.kDifferentialDriveLeft);
    rightMotor = new TalonSRX(1);  //Constants.kDifferentialDriveRight);
    
    leftMotor.setNeutralMode(NeutralMode.Coast);
    rightMotor.setNeutralMode(NeutralMode.Coast);

  }

  public void SetLeftDriveSpeed(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
  }

  public void SetRightDriveSpeed(double speed) {
    rightMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
