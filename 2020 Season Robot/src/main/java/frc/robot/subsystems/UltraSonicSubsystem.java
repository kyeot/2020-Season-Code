/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class UltraSonicSubsystem extends SubsystemBase {

  private final AnalogInput mUltrasonic = new AnalogInput(DriveConstants.kUltrasonicPort);

  public UltraSonicSubsystem() {

  }

  public double GetSensorDistanceInInches() {
    return mUltrasonic.getValue() * DriveConstants.kValueToInches;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Distance to Target1: ","" + GetSensorDistanceInInches());


  }
}
