/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheelSystem;
import frc.robot.subsystems.LEDSubSystem;

public class ColorWheelCommand extends CommandBase {

  private final ColorWheelSystem mColorWheelSubSystem;
  private final LEDSubSystem mLedSubSystem;

  public ColorWheelCommand(ColorWheelSystem colorwheelsystem,LEDSubSystem ledsubsystem) {
    mColorWheelSubSystem = colorwheelsystem;
    mLedSubSystem = ledsubsystem;
    addRequirements(colorwheelsystem);
    //addRequirements(ledsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("ColorWheel Command","Active"  );
    mColorWheelSubSystem.TurnColorWheel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

      SmartDashboard.putString("ColorWheel Command","End"  );
      mColorWheelSubSystem.StopColorWheel();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
