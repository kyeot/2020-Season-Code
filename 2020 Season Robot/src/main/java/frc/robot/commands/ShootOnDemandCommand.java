/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubSystem;
import frc.robot.subsystems.ShooterSubSystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootOnDemandCommand extends CommandBase {

  ShooterSubSystem mShooterSubSystem;
  LEDSubSystem mLedSubSystem;

  private final XboxController mManipulatorController;


  private boolean bMaxReady = false;
  private boolean VomitMode = false;
  private boolean bStopCommand = false;
  private int iMaxCount = 0;
  private double dMaxRPM = 0;
  private double dMotorSpeed = 0;
  private double dStartTime = 0;
  private double dShootingStartTime = 0;

  /**
   * Creates a new ShootOnDemandCommand.
   */
  public ShootOnDemandCommand(ShooterSubSystem shootersubsystem,LEDSubSystem ledsubsystem,XboxController manipulatorcontroller,boolean vomitmode,double speed) {
    mShooterSubSystem = shootersubsystem;
    mLedSubSystem = ledsubsystem;
    mManipulatorController = manipulatorcontroller;
    VomitMode = vomitmode;
    dMotorSpeed = speed;

    addRequirements(shootersubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dMaxRPM =0;
    iMaxCount = 0;
    bMaxReady = false;
    dShootingStartTime =0;
    bStopCommand = false;
    dStartTime = Timer.getFPGATimestamp();
    mLedSubSystem.SetShooterMotorChargingMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dCurrentSpeed = Math.abs( mShooterSubSystem.GetVelocity());

    mShooterSubSystem.SetFeederMotorSpeed(0.25);

    //0.6 = 3188 RPM
    //0.7 = 3645 RPM
    //0.8 = 4085 RPM
    //0.5 = 2588 RPM
 
    /*
     mShooterSubSystem.SetShooterSpeed(dMotorSpeed);
     SmartDashboard.putString("Shooter Velocity: ","" + dCurrentSpeed  );
 
 
 
     // Ramp up configuration Section
     
 
     if (VomitMode == true) {
 
       //  Vomit Priming configuration section
       if (dMaxRPM < dCurrentSpeed)  {
         dMaxRPM = dCurrentSpeed;
       } else {
 
         if ((dMaxRPM/dCurrentSpeed) < 1.1) {
 
           iMaxCount++;
           if (iMaxCount > 40) {
             bMaxReady = true;
           }
         
         } else {
           bMaxReady = false;
           iMaxCount = 0;
         }
       }
 
       if (bMaxReady == true) {
         mLedSubSystem.SetShootingMode();
         //mShooterSubSystem.StartFeederMotor();
       } else {
         mLedSubSystem.SetShooterMotorChargingMode();
         //mShooterSubSystem.StopFeederMotor();
       }

       if ( dShootingStartTime  == 0 && mManipulatorController.getXButtonPressed() ) {
        dShootingStartTime= Timer.getFPGATimestamp();
       } 

       if (dShootingStartTime > 0 ) {
         

         if (Timer.getFPGATimestamp() - dShootingStartTime > 1.5) {
         // mShooterSubSystem.SetShooterSpeed(dMotorSpeed - 0.5 );
         } else if (Timer.getFPGATimestamp() - dShootingStartTime > 0.8) {
          mShooterSubSystem.SetShooterSpeed(dMotorSpeed  );
         } else if (Timer.getFPGATimestamp() - dShootingStartTime > 0.5) {
          mShooterSubSystem.SetShooterSpeed(dMotorSpeed - 0.5 );
         } else {
          mShooterSubSystem.StartFeederMotor();
         }

         if (Timer.getFPGATimestamp() - dShootingStartTime > 3) {
          bStopCommand = true;
         }


       }

 
     } else {
 
       //Timed Ramp Up mode
        if (Timer.getFPGATimestamp() - dStartTime > Constants.kShooterRampTime && dMaxRPM == 0 ) {
          dMaxRPM = dCurrentSpeed;
        }
 
        if (dMaxRPM > 0) {
         if (dCurrentSpeed > dMaxRPM * 0.98) {
           mLedSubSystem.SetShootingMode();
           //mShooterSubSystem.StartFeederMotorSlowMode();
         } else {
           mLedSubSystem.SetShooterMotorChargingMode();
           //mShooterSubSystem.StopFeederMotor();
         }
       }
     }
 
 */
 
     SmartDashboard.putString("Max RPM ","" + dMaxRPM  );
   }
 
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
     SmartDashboard.putString("Shooter Comand End: ","True"  );
     mShooterSubSystem.SetShooterSpeed(0);
     mShooterSubSystem.StopFeederMotor();
     mLedSubSystem.SetRestingMode();
   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
 
     if (bStopCommand) {
       return true;
     } else {
       return false;
     }
   }
 }

 