/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ColorWheelCommand;
import frc.robot.commands.DriveADistanceInFeet;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.ShootOnDemandCommand;
import frc.robot.commands.VisionCommand;
import frc.robot.commands.TurnRight;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.UltrasonicApproachCommand;
import frc.robot.commands.TurnLeft;
import frc.robot.commands.ExtendLiftCommand;
import frc.robot.subsystems.ColorWheelSystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubSystem;
import frc.robot.subsystems.LEDSubSystem;
import frc.robot.subsystems.VisionSubsystem;
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
  private final LEDSubSystem mLEDSubsystem = new LEDSubSystem();
  private final LiftSubSystem mLiftSubsystem = new LiftSubSystem();
  //private final VisionSubsystem mVisionSubsystem = new VisionSubsystem();



  private final XboxController mDriverController = new XboxController(Constants.kDriveController);
  private final XboxController mManipulatorController = new XboxController(Constants.kManipulatorController);
  

  private final DriveCommand mDriveCommand = new DriveCommand(mDriveSubsystem,mDriverController,mManipulatorController,mLEDSubsystem,mLiftSubsystem);

  private final SequentialCommandGroup StraightAutonomous = new SequentialCommandGroup(
    new ShooterCommand(mShooterSubsystem,mLEDSubsystem,mManipulatorController,true,Constants.kShooterSpeedSlow).withTimeout(5),
      new DriveADistanceInFeet(mDriveSubsystem, 35,true).withTimeout(5));


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    mDriveSubsystem.setDefaultCommand(mDriveCommand);
    //SmartDashboard.putData("Autonomous Command", new SequentialCommandGroup(
    //  new DriveADistanceInFeet(mDriveSubsystem, 4,true).withTimeout(5)));

    mLEDSubsystem.SetRestingMode();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

        //new JoystickButton(mDriverController, Button.kX.value)
        //.whenPressed(new VisionCommand(mVisionSubsystem,mDriveSubsystem ).withTimeout(20));

        new JoystickButton(mManipulatorController, Button.kBumperRight.value)
        .whenPressed(new ColorWheelCommand(mColorWheelSubsystem,mLEDSubsystem,mManipulatorController,true).withTimeout(15));

        new JoystickButton(mManipulatorController, Button.kY.value)
         .whenPressed(new ColorWheelCommand(mColorWheelSubsystem,mLEDSubsystem,mManipulatorController,false).withTimeout(15));
        
         new JoystickButton(mManipulatorController, Button.kA.value)
         .whenPressed(new ShooterCommand(mShooterSubsystem,mLEDSubsystem,mManipulatorController,true,Constants.kShooterSpeedSlow).withTimeout(7));

         new JoystickButton(mManipulatorController, Button.kB.value)
         .whenPressed(new ShooterCommand(mShooterSubsystem,mLEDSubsystem,mManipulatorController,true,Constants.kShooterSpeedFast).withTimeout(7));


         //new JoystickButton(mManipulatorController, Button.kA.value)
         //.whenPressed(new ShootOnDemandCommand(mShooterSubsystem,mLEDSubsystem,mManipulatorController,true,Constants.kShootOnDemandSpeedSlow).withTimeout(30));



        // new JoystickButton(mManipulatorController, Button.kBumperRight.value )
       //  .whenPressed(new ShooterCommand(mShooterSubsystem,mLEDSubsystem,mManipulatorController,false,Constants.kShooterSpeedExtraSlow).withTimeout(8));


        //new JoystickButton(mDriverController, Button.kA.value)
        //.whenPressed(new UltrasonicApproachCommand(mDriveSubsystem, mLEDSubsystem).withTimeout(5));

       // new JoystickButton(mDriverController, Button.kX.value)
        //.whenPressed(new VisionCommand(mVisionSubsystem, mDriveSubsystem).withTimeout(20));



       //new JoystickButton(mDriverController, Button.kA.value)
       //.whenPressed(new ExtendLiftCommand(mLiftSubsystem, mLEDSubsystem,mDriverController).withTimeout(15));

        // new JoystickButton(mDriverController, Button.kB.value)
        // .whenPressed(new DriveADistanceInFeet(mDriveSubsystem, 5).withTimeout(5));
       // new JoystickButton(mDriverController, Button.kB.value)
       // .whenPressed(new TurnToTarget(mVisionSubsystem, mDriveSubsystem).withTimeout(3.5));

       // new JoystickButton(mDriverController, Button.kB.value)
        //.whenPressed(new TurnToTarget(mVisionSubsystem, mDriveSubsystem).withTimeout(8));
///

/*
        new JoystickButton(mDriverController,Button.kBumperLeft.value)
        .whenPressed(

        new SequentialCommandGroup(
          new ShooterCommand(mShooterSubsystem,mLEDSubsystem,mManipulatorController,true,Constants.kShooterSpeedSlow).withTimeout(5),
            new TurnRight(mDriveSubsystem,178).withTimeout(2),
            new DriveADistanceInFeet(mDriveSubsystem, 4,false).withTimeout(4),
            new TurnRight(mDriveSubsystem,35).withTimeout(2),
            new DriveADistanceInFeet(mDriveSubsystem, 8,false).withTimeout(10)
            )
        );
        */



  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return  StraightAutonomous;
  }
}
