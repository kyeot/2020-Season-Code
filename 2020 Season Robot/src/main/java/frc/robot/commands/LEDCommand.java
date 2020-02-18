/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubSystem;

public class LEDCommand extends CommandBase {

  LEDSubSystem mLedSubSystem ;
  XboxController mDriveController;


  public LEDCommand(LEDSubSystem ledsubsystem, XboxController drivercontroller) {
    mLedSubSystem = ledsubsystem;
    mDriveController = drivercontroller;
    addRequirements(ledsubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mLedSubSystem.SetLEDMode(-0.87);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //allows for LED's to cycle all solid colors based on right motors current speed
    mLedSubSystem.SetLEDMode(mDriveController.getY(Hand.kRight));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mLedSubSystem.SetLEDMode(0);
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
