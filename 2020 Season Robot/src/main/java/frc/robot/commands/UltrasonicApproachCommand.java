/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubSystem;

public class UltrasonicApproachCommand extends CommandBase {

  DriveSubsystem mDriveSubsystem;
  LEDSubSystem mLEDSubSystem;

  public UltrasonicApproachCommand(DriveSubsystem drivesubsystem,LEDSubSystem ledsubsystem) {
    mDriveSubsystem = drivesubsystem;
    mLEDSubSystem = ledsubsystem;
    addRequirements(drivesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (mDriveSubsystem.GetSensorDistanceInInches() > 15) {
      mDriveSubsystem.SetLeftDriveSpeed(-0.2);
      mDriveSubsystem.SetRightDriveSpeed(-0.2);
    } else 
    {
      mDriveSubsystem.SetLeftDriveSpeed(0);
      mDriveSubsystem.SetRightDriveSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDriveSubsystem.SetLeftDriveSpeed(0);
    mDriveSubsystem.SetRightDriveSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mDriveSubsystem.GetSensorDistanceInInches() > 15) {
      return false;
    } else 
    {
      return true;
    }
    
  }
}
