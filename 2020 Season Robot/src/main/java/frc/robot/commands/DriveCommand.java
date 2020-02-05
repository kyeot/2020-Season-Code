/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ColorWheelSystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.util.NavSensor;

/**
 * An example command that uses an example subsystem.
 */
public class  DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private final ColorWheelSystem m_ColorWheelSystem;
  private final VisionSubsystem m_visionSubsystem;
  private final XboxController m_driverController = new XboxController(0);
  //NavSensor gyro = NavSensor.getInstance();
  private final NavSensor m_gyro = NavSensor.getInstance();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem subsystem, ColorWheelSystem colorwheelsystem, VisionSubsystem visionSystem) {
    m_subsystem = subsystem;
    m_ColorWheelSystem = colorwheelsystem;
    m_visionSubsystem = visionSystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(colorwheelsystem);
    addRequirements(visionSystem);
  }

  public DriveCommand(ColorWheelSystem colorwheelsystem, VisionSubsystem visionSystem) {
    m_ColorWheelSystem = colorwheelsystem;
    m_visionSubsystem = visionSystem;
    m_subsystem  = null;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(colorwheelsystem);
    addRequirements(visionSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speedLeft = m_driverController.getY(Hand.kLeft);
    double speedRight= m_driverController.getY(Hand.kRight);

    SmartDashboard.putString("left","" + speedLeft);
    SmartDashboard.putString("right","" + speedRight);
    SmartDashboard.putString("l Distance","" + m_subsystem.getAverageEncoderDistance() );
    SmartDashboard.putString("Nav X angle","" + m_gyro.getRawAngle());


    //m_subsystem.SetLeftDriveSpeed(speedLeft);
    //m_subsystem.SetRightDriveSpeed(speedRight);

    SmartDashboard.putString("DB/String 1", "Target Angle: " + m_visionSubsystem.getCenterX());
    SmartDashboard.putString("DB/String 2", "TL: " + m_visionSubsystem.getBoundingRect().tl());
    SmartDashboard.putString("DB/String 3", "BR: " + m_visionSubsystem.getBoundingRect().br());
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
