/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveADistanceInFeet;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TurnRight90;
import frc.robot.commands.TurnLeft90;
import frc.robot.subsystems.ColorWheelSystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants;



/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();
  private final ColorWheelSystem mColorWheelSubsystem = new ColorWheelSystem();
  private final ShooterSubSystem mShooterSubsystem = new ShooterSubSystem();
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();


  private final XboxController mDriverController = new XboxController(Constants.kDriveController);

  private final DriveCommand m_autoCommand = new DriveCommand(mDriveSubsystem,mDriverController);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    mDriveSubsystem.setDefaultCommand(new DriveCommand(mDriveSubsystem,mDriverController));


  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
    new JoystickButton(mDriverController, Button.kX.value)
        .whenPressed(new TurnLeft90(mDriveSubsystem).withTimeout(8));

        
        new JoystickButton(mDriverController, Button.kA.value)
        .whenPressed(new ShooterCommand(mShooterSubsystem).withTimeout(5));

        new JoystickButton(mDriverController, Button.kB.value)
        .whenPressed(new DriveADistanceInFeet(mDriveSubsystem, 5).withTimeout(5));

        new JoystickButton(mDriverController, Button.kY.value)
        .whenPressed(

          new SequentialCommandGroup(
            new DriveADistanceInFeet(mDriveSubsystem, 10).withTimeout(10),
            new TurnLeft90(mDriveSubsystem).withTimeout(8),
            new DriveADistanceInFeet(mDriveSubsystem, 10).withTimeout(10),
            new TurnLeft90(mDriveSubsystem).withTimeout(8),
            new ShooterCommand(mShooterSubsystem).withTimeout(5)
            )
            
        );


         /*
    new JoystickButton(mDriverController, Button.kA.value)
        .whenPressed(() -> m_robotArm.setGoal(2), m_robotArm);

       
    // Move the arm to neutral position when the 'B' button is pressed.
    new JoystickButton(m_driverController, Button.kB.value)
        .whenPressed(() -> m_robotArm.setGoal(Constants.ArmConstants.kArmOffsetRads), m_robotArm);

    // Drive at half speed when the bumper is held
    new JoystickButton(m_driverController, Button.kBumperRight.value)
        .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
        .whenReleased(() -> m_robotDrive.setMaxOutput(1));
    */

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return  m_autoCommand;
  }
}
