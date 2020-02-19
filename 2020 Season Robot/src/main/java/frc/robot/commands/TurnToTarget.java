/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  
  DecimalFormat df = new DecimalFormat("#.###");
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (initialDegrees < 0) {
      ds.SetRightDriveSpeed(0.15);
      ds.SetLeftDriveSpeed(-0.15);
    } else if (initialDegrees > 0) {
      ds.SetRightDriveSpeed(-0.15);
      ds.SetLeftDriveSpeed(0.15);
    }

    SmartDashboard.putString("DB/String 1", "Target Angle: " + df.format(vs.getRawAngle()));
    SmartDashboard.putString("DB/String 2", "Center X: " + vs.getCenterX());
    SmartDashboard.putString("DB/String 3", "TL: " + vs.getBoundingRect().tl());
    SmartDashboard.putString("DB/String 4", "BR: " + vs.getBoundingRect().br());
    SmartDashboard.putString("DB/String 5", "Distance (in): " + vs.getDistance());
    SmartDashboard.putString("DB/String 6", "Gyro: " + df.format(ds.getHeading()));
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
        return true;
      } else {
        return false;
      }
    } else if (initialDegrees > 0) {
      if (ds.getHeading() > initialDegrees) {
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