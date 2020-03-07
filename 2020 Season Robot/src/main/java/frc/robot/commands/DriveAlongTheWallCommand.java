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

      CalculateDriveModifiers( );
      
      SmartDashboard.putString("leftDriveSpeed","s " + dLeftDriveSpeed  );
      SmartDashboard.putString("rightDriveSpeed","s "  + dRightDriveSpeed );

      if (bReverse) {
        mDriveSubsystem.SetLeftDriveSpeed(-dRightDriveSpeed );
        mDriveSubsystem.SetRightDriveSpeed(-dLeftDriveSpeed );
      } else {
        mDriveSubsystem.SetRightDriveSpeed(-dRightDriveSpeed );
        mDriveSubsystem.SetLeftDriveSpeed(-dLeftDriveSpeed );
      }

    }
  }

  private void CalculateDriveModifiers() {
     
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
          SmartDashboard.putString("Wall Status ","straight"    );
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
          
          if (kHoldDistance < dNewDistanceToTarget)
          {
            //To Far
            if (dLastDistanceToTarget > dNewDistanceToTarget) 
            {
              if (dLastDistanceToTarget - dNewDistanceToTarget > 0.1) {
                SmartDashboard.putString("Wall Status ","Far->.1"    );
                dLeftDriveSpeed = dLeftDriveSpeed + (dLeftDriveSpeed * 0.05);
              } else {
                SmartDashboard.putString("Wall Status ","Far-<.1"    );
                dRightDriveSpeed = dRightDriveSpeed + (dRightDriveSpeed * 0.02);
              }
    
            } 
            else 
            {
              //wrong way
              SmartDashboard.putString("Wall Status ","Far-Wrong way"    );
              dLeftDriveSpeed = dLeftDriveSpeed * 0.7;
              dRightDriveSpeed = dRightDriveSpeed * 1.2;
            }
          }
          else
          {
            //To Close 
            if (dLastDistanceToTarget < dNewDistanceToTarget) 
            {
              if (dNewDistanceToTarget - dLastDistanceToTarget > 0.1) {
                SmartDashboard.putString("Wall Status ","Close->.1"    );
                dRightDriveSpeed = dRightDriveSpeed + (dRightDriveSpeed * 0.05);
              } else {
                SmartDashboard.putString("Wall Status ","Close-<.1"    );
                dLeftDriveSpeed= dLeftDriveSpeed + (dLeftDriveSpeed * 0.02);
              }
            }
            else
            {
              //wrong way
              SmartDashboard.putString("Wall Status ","CLose-Wrong way"    );
              dLeftDriveSpeed = dLeftDriveSpeed * 1.3;
              dRightDriveSpeed = dRightDriveSpeed * 0.7;
            }
          }


        }

       // if (Math.abs(dRightDriveSpeed) - Math.abs(dRightDriveSpeed))

        dLastDistanceToTarget = dNewDistanceToTarget;
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
