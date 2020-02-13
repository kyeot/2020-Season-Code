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
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;

public class DriveSubsystem extends SubsystemBase {

  
	TalonSRX leftMotor;
  TalonSRX rightMotor;

  VictorSPX mLeftMotorV1;
  VictorSPX mRightMotorV1;
  VictorSPX mLeftMotorV2;
  VictorSPX mRightMotorV2;


  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final AnalogInput mUltrasonic = new AnalogInput(DriveConstants.kUltrasonicPort);

  private final Encoder mLeftEncoder  = new Encoder(0, 1, DriveConstants.kLeftEncoderReversed);;
  private final Encoder mRightEncoder = new Encoder(2, 3, DriveConstants.kRightEncoderReversed);;


  public DriveSubsystem() {

		//leftMotor = new TalonSRX(11); //Constants.kDifferentialDriveLeft);
    //rightMotor = new TalonSRX(13);  //Constants.kDifferentialDriveRight);

    try {
      mLeftMotorV1 = new VictorSPX(DriveConstants.kLeftMotor1Port);
      mLeftMotorV2 = new VictorSPX(DriveConstants.kLeftMotor2Port);
      mRightMotorV1 = new VictorSPX(DriveConstants.kRightMotor1Port);
      mRightMotorV2 = new VictorSPX(DriveConstants.kRightMotor2Port);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error Drive Motor Controllers:  " + ex.getMessage(), true);
    }
    

    mLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    mRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);




     
    mLeftMotorV1.setNeutralMode(NeutralMode.Brake);
    mLeftMotorV2.setNeutralMode(NeutralMode.Brake);
    mRightMotorV1.setNeutralMode(NeutralMode.Brake);
    mRightMotorV2.setNeutralMode(NeutralMode.Brake);


  }

  public void SetLeftDriveSpeed(double speed) {
    //leftMotor.set(ControlMode.PercentOutput, speed);
    mLeftMotorV1.set(ControlMode.PercentOutput, speed);
    mLeftMotorV2.set(ControlMode.PercentOutput, speed);

    //LEDPort1.setSpeed(speed);

  }

  public void SetRightDriveSpeed(double speed) {
    //rightMotor.set(ControlMode.PercentOutput, speed);
    mRightMotorV1.set(ControlMode.PercentOutput, -speed);
    mRightMotorV2.set(ControlMode.PercentOutput, -speed);

  }

  public void resetEncoders() {
    mLeftEncoder.reset();
    mRightEncoder.reset();
  }

  public double getAverageEncoderDistance() {
    return ((mLeftEncoder.getDistance()  + mRightEncoder.getDistance()) / 2.0);
  }

  public double getLeftEncoderDistance() {
    return (mLeftEncoder.getDistance());
  }

  public double getRightEncoderDistance() {
    return ( mRightEncoder.getDistance());
  }

    /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    SmartDashboard.putString("Yaw: ","" + gyro.getYaw()  );
    return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double GetSensorDistanceInInches() {
    return mUltrasonic.getValue() * DriveConstants.kValueToInches;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
