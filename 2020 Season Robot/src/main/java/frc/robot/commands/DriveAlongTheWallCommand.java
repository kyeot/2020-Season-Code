/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.UltraSonicSubsystem;

public class DriveAlongTheWallCommand extends CommandBase {

  private final DriveSubsystem mDriveSubsystem;
  private final UltraSonicSubsystem mUltraSonicSubsystem;
  private final double mDistanceInFeet;
  private boolean bFirstRun;
  private boolean bReverse;
  int kHoldDistance = 40;
  double dLastTime = 0;
  double dLastDistanceToTarget = 0;
  double dRightDriveSpeed= 0;
  double dLeftDriveSpeed=0;

  public DriveAlongTheWallCommand(DriveSubsystem drive, UltraSonicSubsystem ultrasonicsubsystem, double distanceinfeet, boolean reverse) {
    mDistanceInFeet = distanceinfeet;
    bReverse = reverse;
    mDriveSubsystem = drive;
    mUltraSonicSubsystem = ultrasonicsubsystem;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDriveSubsystem.resetEncoders();
    if (bReverse) {
      dLeftDriveSpeed = -Constants.DriveConstants.kAutonomousDriveSpeed;
      dRightDriveSpeed = -Constants.DriveConstants.kAutonomousDriveSpeed;
    } else {
      dLeftDriveSpeed = Constants.DriveConstants.kAutonomousDriveSpeed;
      dRightDriveSpeed = Constants.DriveConstants.kAutonomousDriveSpeed;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dDriveSpeed;



    //robotdrive.drive(SPEED, Gyro.getAngle() * .03);
    if (bFirstRun == true) {
      SmartDashboard.putString("Initial Heading: ","" + mDriveSubsystem.getHeading()  );
      mDriveSubsystem.SetRightDriveSpeed(-dRightDriveSpeed  );
      mDriveSubsystem.SetLeftDriveSpeed(-dLeftDriveSpeed );
      bFirstRun = false;
    }
    else {
      SmartDashboard.putString("Commnad Heading: ","" + mDriveSubsystem.getHeading()  );
      mDriveSubsystem.SetRightDriveSpeed(-dRightDriveSpeed + (Constants.DriveConstants.kAutonomousDriveSpeed * mDriveSubsystem.getHeading() * .03 ));
      mDriveSubsystem.SetLeftDriveSpeed(-dLeftDriveSpeed );
    }
  }

  private void CalculateDriveModifiers(double leftDriveSpeed, double rightDriveSpeed) {
     
    if (bFirstRun ) 
    {
      dLastTime =Timer.getFPGATimestamp();
      dLastDistanceToTarget = mUltraSonicSubsystem.GetSensorDistanceInInches();
    } 
    else 
    {
      if (Timer.getFPGATimestamp() - dLastTime > 0.1) 
      {
        dLastTime =Timer.getFPGATimestamp();
        double dNewDistanceToTarget = mUltraSonicSubsystem.GetSensorDistanceInInches();
        
        if (Math.abs(kHoldDistance - dNewDistanceToTarget ) < 1 && Math.abs(dLastDistanceToTarget - dNewDistanceToTarget) < 0.05 ) 
        {

          if (bReverse) 
          {
            dLeftDriveSpeed = -Constants.DriveConstants.kAutonomousDriveSpeed;
            dRightDriveSpeed = -Constants.DriveConstants.kAutonomousDriveSpeed;
          } else 
          {
            dLeftDriveSpeed = Constants.DriveConstants.kAutonomousDriveSpeed;
            dRightDriveSpeed = Constants.DriveConstants.kAutonomousDriveSpeed;
          }

        } 
        else 
        {
         // need to check if we are too close or too far before these calcuations are made

          if (dLastDistanceToTarget > dNewDistanceToTarget) 
          {
            if (dLastDistanceToTarget - dNewDistanceToTarget > 0.1) {
              leftDriveSpeed = leftDriveSpeed + (leftDriveSpeed * 0.5);
            } else {
              rightDriveSpeed = rightDriveSpeed + (rightDriveSpeed * 0.2);
            }
  
          } else 
          {
            if (dNewDistanceToTarget - dLastDistanceToTarget > 0.1) {
              rightDriveSpeed = rightDriveSpeed + (rightDriveSpeed * 0.5);
            } else {
              leftDriveSpeed = leftDriveSpeed + (leftDriveSpeed * 0.2);
            }
  
          }
        }

      }

   //   add saftety if left or right delta too big

//if distance less than 29, hard turn

    }
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
