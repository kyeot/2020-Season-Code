/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubSystem;
import frc.robot.subsystems.LiftSubSystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ExtendLiftCommand extends CommandBase {

  LiftSubSystem mLiftSubSystem;
  LEDSubSystem mLedSubSystem;
  XboxController mControler;

  public ExtendLiftCommand(LiftSubSystem liftsubsystem, LEDSubSystem ledsubsystem,XboxController controller) {

    mLiftSubSystem = liftsubsystem;
    mLedSubSystem = ledsubsystem;
    mControler = controller;

    //addRequirements(liftsubsystem);



    //Port 4 - Encoder

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mLiftSubSystem.ResetEncoder();
    //mLedSubSystem.SetLEDMode(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Lift Command End","False"  );
    mLiftSubSystem.SetLiftSpeed(-0.7);
    SmartDashboard.putString("Encoder","" + mLiftSubSystem.GetEncoderDistance() );
    /*
    if (mLiftSubSystem.isSwitchSet() == true) {
      mLedSubSystem.SetLEDMode(0.7);
    }
    */
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Lift Command End","True"  );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mLiftSubSystem.GetEncoderDistance() >0.74 ) {
      return true;
    } else {}
      return false;
    }
  }
}
