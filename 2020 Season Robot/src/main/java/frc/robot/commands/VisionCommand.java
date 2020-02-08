/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubSystem;

public class VisionCommand extends CommandBase {
  /**
   * Creates a new VisionCommand.
   */

  private VisionSubSystem visionSubsystem;
  private DriveSubsystem driveSubsystem;

  public VisionCommand(VisionSubSystem vs,DriveSubsystem ds) {
    // Use addRequirements() here to declare subsystem dependencies.
    visionSubsystem = vs;
    driveSubsystem = ds;
    addRequirements(vs);
    addRequirements(ds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("DB/String 1", "Center X: " + visionSubsystem.getCenterX());
    SmartDashboard.putString("DB/String 2", "Top Left point: " + visionSubsystem.getBoundingRect().tl());
    SmartDashboard.putString("DB/String 3", "Bottom Right point: " + visionSubsystem.getBoundingRect().br());


    //driveSubsystem.  here is the fun part
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}