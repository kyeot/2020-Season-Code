/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class LimeTurnToTarget extends CommandBase {

  final double STEER_K = -0.03;                    // how hard to turn toward the target
  final double MIN_COMMAND_K = 0.05;
  final double MAX_SPEED = 3;


  private final DriveSubsystem mDriveSubsystem;

  public LimeTurnToTarget(DriveSubsystem drivesubsystem) {
    mDriveSubsystem = drivesubsystem;
    addRequirements(drivesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    //double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    //double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    SmartDashboard.putString("DB/String 0", "tv:" + tv);
    SmartDashboard.putString("DB/String 1", "tx:" + tx);

    double dLeftSpeed = 0;
    double dRightSpeed = 0;

    if (tv >= 1.0)
    {

      double heading_error = -tx;
      double steering_adjust = 0.0;
      if (tx > 1.0)
      {
              steering_adjust = STEER_K * heading_error - MIN_COMMAND_K;
      }
      else if (tx < 1.0)
      {
              steering_adjust = STEER_K * heading_error + MIN_COMMAND_K;
      }
      dLeftSpeed += steering_adjust;
      dRightSpeed -= steering_adjust;
    }

    if (Math.abs(dLeftSpeed) > MAX_SPEED) {
      if (dLeftSpeed > 0) {
        dLeftSpeed = MAX_SPEED;
      } else {
        dLeftSpeed = -MAX_SPEED;
      }
    }

    if (Math.abs(dRightSpeed) > MAX_SPEED) {
      if (dRightSpeed > 0) {
        dRightSpeed = MAX_SPEED;
      } else {
        dRightSpeed = -MAX_SPEED;
      }
    }

    mDriveSubsystem.SetLeftDriveSpeed(dLeftSpeed);
    mDriveSubsystem.SetRightDriveSpeed(dRightSpeed);



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
