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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveADistanceInFeet extends CommandBase {

  private final DriveSubsystem mDriveSubsystem;
  private final double mDistanceInFeet;
  private boolean bFirstRun;
  private boolean bReverse;

  public DriveADistanceInFeet(DriveSubsystem drive, double distanceinfeet,boolean reverse) {

    mDistanceInFeet = distanceinfeet;
    bReverse = reverse;
    mDriveSubsystem = drive;
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bFirstRun = true;
    mDriveSubsystem.resetEncoders();
    mDriveSubsystem.zeroHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double dDriveSpeed;

    if (bReverse) {
      dDriveSpeed = -Constants.DriveConstants.kAutonomousDriveSpeed;
    } else {
      dDriveSpeed = Constants.DriveConstants.kAutonomousDriveSpeed;
    }

    //robotdrive.drive(SPEED, Gyro.getAngle() * .03);
    if (bFirstRun == true) {
      SmartDashboard.putString("Initial Heading: ","" + mDriveSubsystem.getHeading()  );
      mDriveSubsystem.SetRightDriveSpeed(-Constants.DriveConstants.kAutonomousDriveSpeed  );
      mDriveSubsystem.SetLeftDriveSpeed(-Constants.DriveConstants.kAutonomousDriveSpeed );
      bFirstRun = false;
    }
    else {
      SmartDashboard.putString("Commnad Heading: ","" + mDriveSubsystem.getHeading()  );
      mDriveSubsystem.SetRightDriveSpeed(-Constants.DriveConstants.kAutonomousDriveSpeed + (Constants.DriveConstants.kAutonomousDriveSpeed * mDriveSubsystem.getHeading() * .03 ));
      mDriveSubsystem.SetLeftDriveSpeed(-Constants.DriveConstants.kAutonomousDriveSpeed );
    }



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (Math.abs(mDriveSubsystem.getAverageEncoderDistance()) / Constants.DriveConstants.kEncoderFootPerDistance > mDistanceInFeet  ) {
      return true;
    }
    else {
      return false;
    }

    
  }
}
