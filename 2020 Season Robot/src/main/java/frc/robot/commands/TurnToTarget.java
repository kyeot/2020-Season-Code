/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TurnToTarget extends CommandBase {
  /**
   * Creates a new TurnToTarget.
   */
  private VisionSubsystem vs;
  private DriveSubsystem ds;
  private double initialDegrees;

  public TurnToTarget(VisionSubsystem vs, DriveSubsystem ds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ds = ds;
    this.vs = vs;
    addRequirements(vs);
    addRequirements(ds);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ds.zeroHeading();
    initialDegrees = vs.getRawAngle();
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (initialDegrees < 0) {
      ds.SetRightDriveSpeed(0.2);
      ds.SetLeftDriveSpeed(-0.2);
    } else if (initialDegrees > 0) {
      ds.SetRightDriveSpeed(-0.2);
      ds.SetLeftDriveSpeed(0.2);
    }
    if (finalIteration) {
      if (initialDegrees < 0) {
        ds.SetRightDriveSpeed(-0.5);
        ds.SetLeftDriveSpeed(0.5);
      } else if (initialDegrees > 0) {
        ds.SetRightDriveSpeed(0.5);
        ds.SetLeftDriveSpeed(-0.5);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ds.SetRightDriveSpeed(0);
    ds.SetLeftDriveSpeed(0);
  }

  // Returns true when the command should end.
  int count = 0;
  boolean finalIteration = false;
  @Override
  public boolean isFinished() {
    if (initialDegrees < 0) {
      if (ds.getHeading() < initialDegrees) {
        finalIteration = true;
        return true;
      } else {
        return false;
      }
    } else if (initialDegrees > 0) {
      if (ds.getHeading() > initialDegrees) {
        finalIteration = true;
        return true;
      } else {
        return false;
      }
    } else {
      //If initialDegrees = 0
      return true;
    }
  }
}
