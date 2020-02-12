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
import frc.robot.subsystems.VisionSubsystem;

public class VisionCommand extends CommandBase {
  /**
   * Creates a new VisionCommand.
   */

  private VisionSubsystem visionSubsystem;
  private DriveSubsystem driveSubsystem;

  public VisionCommand(VisionSubsystem vs,DriveSubsystem ds) {
    // Use addRequirements() here to declare subsystem dependencies.
    visionSubsystem = vs;
    addRequirements(vs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putString("DB/String 1", "Target Angle: " + visionSubsystem.getRawAngle());
    SmartDashboard.putString("DB/String 2", "Center X: " + visionSubsystem.getCenterX());
    SmartDashboard.putString("DB/String 3", "TL: " + visionSubsystem.getBoundingRect().tl());
    SmartDashboard.putString("DB/String 4", "BR: " + visionSubsystem.getBoundingRect().br());
    SmartDashboard.putString("DB/String 5", "Distance (in): " + visionSubsystem.getDistance());


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
