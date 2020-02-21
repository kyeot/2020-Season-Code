/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubSystem;
import frc.robot.subsystems.ShooterSubSystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterCommand extends CommandBase {

  ShooterSubSystem mShooterSubSystem;
  LEDSubSystem mLedSubSystem;
  private double dMaxRPM = 0;
  private boolean bMaxReady = false;
  private int iMaxCount = 0;

  public ShooterCommand(ShooterSubSystem shootersubsystem,LEDSubSystem ledsubsystem) {
    mShooterSubSystem = shootersubsystem;
    mLedSubSystem = ledsubsystem;
    addRequirements(shootersubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // mShooterSubSystem.StartFeederMotor();
   dMaxRPM =0;
   iMaxCount = 0;
   bMaxReady = false;
   mLedSubSystem.SetShooterMotorChargingMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dCurrentSpeed = Math.abs( mShooterSubSystem.GetVelocity());

    mShooterSubSystem.SetShooterSpeed(-0.3);
    //mShooterSubSystem.StartFeederMotor();
    //mShooterSubSystem.SetMotorRPM(3000);
    //mShooterSubSystem.SetShooterSpeed(0.4);
    SmartDashboard.putString("Shooter Velocity: ","" + dCurrentSpeed  );

    
    if (dMaxRPM < dCurrentSpeed)  {
      dMaxRPM = dCurrentSpeed;
    } else {

      if ((dMaxRPM/dCurrentSpeed) < 1.04) {

        iMaxCount++;
        if (iMaxCount > 10) {
          bMaxReady = true;
        }
        
      } else {
        bMaxReady = false;
        iMaxCount = 0;
      }
    }

    if (bMaxReady == true) {
      
      mLedSubSystem.SetShootingMode();
      mShooterSubSystem.StartFeederMotor();
    } else {
      mLedSubSystem.SetShooterMotorChargingMode();
      mShooterSubSystem.StopFeederMotor();
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Shooter Comand End: ","True"  );
    mShooterSubSystem.SetShooterSpeed(0);
    mShooterSubSystem.StopFeederMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
