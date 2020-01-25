/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;


/**
 * An example command that uses an example subsystem.
 */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem mDriveSubSystem;
  private final XboxController mDriverController; 


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand( DriveSubsystem drivesubsystem, XboxController drivercontroller) {
    mDriveSubSystem = drivesubsystem;
    mDriverController = drivercontroller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivesubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    mDriveSubSystem.resetEncoders();
    mDriveSubSystem.zeroHeading();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speedLeft = mDriverController.getY(Hand.kLeft );
    double speedRight= mDriverController.getY(Hand.kRight);

    SmartDashboard.putString("left","" + speedLeft );
    SmartDashboard.putString("right","" + speedRight );
    SmartDashboard.putString("l Distance","" + mDriveSubSystem.getLeftEncoderDistance() );
    SmartDashboard.putString("r Distance","" + mDriveSubSystem.getRightEncoderDistance() );
    SmartDashboard.putString("Heading: ","" + mDriveSubSystem.getHeading());


    mDriveSubSystem.SetLeftDriveSpeed(-speedLeft);
    mDriveSubSystem.SetRightDriveSpeed(-speedRight);


    //m_ColorWheelSystem.ReadColorSensor();


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
