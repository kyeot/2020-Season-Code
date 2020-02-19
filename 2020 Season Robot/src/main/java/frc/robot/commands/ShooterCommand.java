/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubSystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterCommand extends CommandBase {

  ShooterSubSystem mShooterSubSystem;

  public ShooterCommand(ShooterSubSystem shootersubsystem) {
    mShooterSubSystem = shootersubsystem;

    addRequirements(shootersubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // mShooterSubSystem.StartFeederMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //mShooterSubSystem.SetShooterSpeed(0.6);
    mShooterSubSystem.SetMotorRPM(3000);
    //mShooterSubSystem.SetShooterSpeed(0.4);
    SmartDashboard.putString("Shooter Velocity: ","" + mShooterSubSystem.GetVelocity()  );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Shooter Comand End: ","True"  );
    //mShooterSubSystem.SetShooterSpeed(0);
   // mShooterSubSystem.StopFeederMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
