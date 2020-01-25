/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveADistanceInFeet extends CommandBase {

  private final DriveSubsystem mDriveSubsystem;
  private final double mDistanceInFeet;

  public DriveADistanceInFeet(DriveSubsystem drive, double distanceinfeet) {

    mDistanceInFeet = distanceinfeet;

    mDriveSubsystem = drive;
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDriveSubsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDriveSubsystem.SetRightDriveSpeed(-0.2);
    mDriveSubsystem.SetLeftDriveSpeed(-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (mDriveSubsystem.getAverageEncoderDistance() / Constants.DriveConstants.kEncoderFootPerDistance > mDistanceInFeet  ) {
      return true;
    }
    else {
      return false;
    }

    
  }
}
