/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants.DriveConstants;


public class DriveSubsystem extends SubsystemBase {

  
	TalonSRX leftMotor;
  TalonSRX rightMotor;

  VictorSPX leftMotorV1;
  VictorSPX rightMotorV1;
  VictorSPX leftMotorV2;
  VictorSPX rightMotorV2;



  private final Encoder m_leftEncoder = new Encoder(0, 1, DriveConstants.kLeftEncoderReversed);

// The right-side drive encoder
//private final Encoder m_rightEncoder =
  //new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1],
    //          DriveConstants.kRightEncoderReversed);



  public DriveSubsystem() {

		//leftMotor = new TalonSRX(11); //Constants.kDifferentialDriveLeft);
    //rightMotor = new TalonSRX(13);  //Constants.kDifferentialDriveRight);

    leftMotorV1 = new VictorSPX(DriveConstants.kLeftMotor1Port);
    leftMotorV2 = new VictorSPX(DriveConstants.kLeftMotor2Port);
    rightMotorV1 = new VictorSPX(DriveConstants.kRightMotor1Port);
    rightMotorV2 = new VictorSPX(DriveConstants.kRightMotor2Port);

    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    //m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
     
    leftMotorV1.setNeutralMode(NeutralMode.Coast);
    leftMotorV2.setNeutralMode(NeutralMode.Coast);
    rightMotorV1.setNeutralMode(NeutralMode.Coast);
    rightMotorV2.setNeutralMode(NeutralMode.Coast);

  }

  public void SetLeftDriveSpeed(double speed) {
    //leftMotor.set(ControlMode.PercentOutput, speed);
    leftMotorV1.set(ControlMode.PercentOutput, -speed);
    leftMotorV2.set(ControlMode.PercentOutput, -speed);
  }

  public void SetRightDriveSpeed(double speed) {
    //rightMotor.set(ControlMode.PercentOutput, speed);
    rightMotorV1.set(ControlMode.PercentOutput, speed);
    rightMotorV2.set(ControlMode.PercentOutput, speed);
  }

  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance()); // + m_rightEncoder.getDistance()) / 2.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
