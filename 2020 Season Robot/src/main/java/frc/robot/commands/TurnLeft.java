/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnLeft extends CommandBase {

  private final DriveSubsystem mDriveSubsystem;
  private int iDegrees = 0;

  public TurnLeft(DriveSubsystem drive,int degrees) {

    iDegrees = degrees;

    mDriveSubsystem = drive;
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDriveSubsystem.zeroHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    mDriveSubsystem.SetRightDriveSpeed(0.3);
    mDriveSubsystem.SetLeftDriveSpeed(-0.3);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mDriveSubsystem.getHeading() > -iDegrees){
      return false;
    }
    else {
      mDriveSubsystem.SetRightDriveSpeed(0);
      mDriveSubsystem.SetLeftDriveSpeed(0);
      return true;
    }
    
  }}
