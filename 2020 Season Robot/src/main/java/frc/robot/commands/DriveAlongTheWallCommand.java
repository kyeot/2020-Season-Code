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
      mDriveSubsystem.SetRightDriveSpeed(dDriveSpeed  );
      mDriveSubsystem.SetLeftDriveSpeed(dDriveSpeed );
      bFirstRun = false;
    }
    else {
      SmartDashboard.putString("Commnad Heading: ","" + mDriveSubsystem.getHeading()  );

      //CalculateDriveModifiers( );
      double dNewDistanceToTarget = mUltraSonicSubsystem.GetSensorDistanceInInches();
      double dDeltaDistance = kHoldDistance - dNewDistanceToTarget;
      double dSteerSpeed = dDriveSpeed = dDriveSpeed + (Constants.DriveConstants.kAutonomousDriveSpeed * dDeltaDistance * .02 );
      
      dLastDistanceToTarget = dNewDistanceToTarget;

      if (bReverse) {
        mDriveSubsystem.SetLeftDriveSpeed(dDriveSpeed );
        mDriveSubsystem.SetRightDriveSpeed(dSteerSpeed );
      } else {
        mDriveSubsystem.SetLeftDriveSpeed(dSteerSpeed );
        mDriveSubsystem.SetRightDriveSpeed(dDriveSpeed);
      }

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
