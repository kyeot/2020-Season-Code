/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubSystem;
import frc.robot.subsystems.ShooterSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DefaultAutonomous extends SequentialCommandGroup {
  /**
   * Creates a new DefaultAutonomous.
   */
  public DefaultAutonomous(DriveSubsystem ds,
                           ShooterSubSystem ss,
                           LEDSubSystem ls,
                           XboxController manipulator,
                           double distanceInFeet) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new ShooterCommand(ss, ls, manipulator, true, Constants.kShooterSpeedSlow).withTimeout(5),
          new DriveADistanceInFeet(ds, distanceInFeet, true).withTimeout(5));
  }
}
