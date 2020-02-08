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
import edu.wpi.first.wpilibj.XboxController.Button;


/**
 * An example command that uses an example subsystem.
 */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem mDriveSubSystem;
  private final XboxController mDriverController; 
  private boolean bReverseDrive = false;

  double leftSpeed;
	double rightSpeed;
	boolean lastButton1State = false;
  boolean reverseButton1Toggle = false;
  boolean biggerRight;
	boolean goingForward;
  


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

    double scale;
		
		if (mDriverController.getBumperPressed(Hand.kLeft)) {
			//Quarter speed
			scale = 0.25;
		} 
		
		else if (mDriverController.getBumperPressed(Hand.kRight)) {
			//Full speed
			scale = 1;
		}
		
		else {
			//Default speed of .75
			scale = 0.75;
		}


		setSpeeds(scale);
		checkStationaryRotation(scale);
			
		if (Math.abs(leftSpeed) < 0.15) {
			leftSpeed = 0;
		}

		if (Math.abs(rightSpeed) < 0.15) {
			rightSpeed = 0;
		}

    //Backwards Driver Drive
    /*
		if(OI.driver.getRawButton(Constants.kBackwardsDrive) == true && lastButton1State == false) {
			reverseButton1Toggle = toggleInput(reverseButton1Toggle);
			lastButton1State = true;
		} else if (OI.driver.getRawButton(Constants.kBackwardsDrive) == false) {
			lastButton1State = false;
    }
    */

    mDriveSubSystem.SetLeftDriveSpeed(-leftSpeed);
    mDriveSubSystem.SetRightDriveSpeed(-rightSpeed);
		
		//if(reverseButton1Toggle) {
		//	Robot.tankDrive.tankDrive(-rightSpeed, -leftSpeed);
		//}
		//else{
		//	Robot.tankDrive.tankDrive(leftSpeed, rightSpeed);
		//}


    /*
    double speedLeft = mDriverController.getY(Hand.kLeft ) * scale ;
    double speedRight= mDriverController.getY(Hand.kRight)  * scale ;

    //mDriverController.

    //Button.kX.value


    SmartDashboard.putString("left","" + speedLeft );
    SmartDashboard.putString("right","" + speedRight );
    SmartDashboard.putString("l Distance","" + mDriveSubSystem.getLeftEncoderDistance() );
    SmartDashboard.putString("r Distance","" + mDriveSubSystem.getRightEncoderDistance() );
    SmartDashboard.putString("Heading: ","" + mDriveSubSystem.getHeading());


    if (!bReverseDrive)
    {
      mDriveSubSystem.SetLeftDriveSpeed(-speedLeft);
      mDriveSubSystem.SetRightDriveSpeed(-speedRight);
    }
    else 
    {
      mDriveSubSystem.SetLeftDriveSpeed(speedLeft);
      mDriveSubSystem.SetRightDriveSpeed(speedRight);
    }

    */
  
    //m_ColorWheelSystem.ReadColorSensor();


  }

  public void SetReverseDrive () {
    bReverseDrive = !bReverseDrive;
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

  	// Called repeatedly when this Command is scheduled to run
	public double averageWheelOutput(double lTrigger, double rTrigger) {
		return -(rTrigger - lTrigger);
	}
	
	public boolean isNegative(double value) {
		return value < 0;
	}
	

	
	public double scaleSide(char side, double initialOutput, double angularValue) {
		angularValue = -angularValue;
		biggerRight = isNegative(angularValue);
		goingForward = isNegative(initialOutput);
		if (goingForward) {
			if (biggerRight) {
				if (side == 'l') {
					return initialOutput;
				} else {
					return (initialOutput + initialOutput*angularValue);
				}
			} else {
				if (side == 'l') {
					return initialOutput - initialOutput*angularValue;
				} else {
					return initialOutput;
				}
			}
		} else {
			if (biggerRight) {
				if (side == 'l') {
					return initialOutput;
				} else {
					return (initialOutput - initialOutput*angularValue);
				}
			} else {
				if (side == 'l') {
					return initialOutput + initialOutput*angularValue;
				} else {
					return initialOutput;
				}
			}
		}
	}
	
	public void setSpeeds(double scale) {
    

		leftSpeed = scale*scaleSide('l', averageWheelOutput(mDriverController.getRawAxis(3), mDriverController.getRawAxis(2)), mDriverController.getRawAxis(0));
		rightSpeed = scale*scaleSide('r', averageWheelOutput(mDriverController.getRawAxis(3), mDriverController.getRawAxis(2)), mDriverController.getRawAxis(0));
	}	
	
	public void checkStationaryRotation(double scale) {
		if (scale == .75) {
			scale = .5;
		}
		if (/*Math.abs(OI.driver.getRawAxis(0)) > .25 &&*/ mDriverController.getRawAxis(3) < .15 && mDriverController.getRawAxis(2) < .15) {
			leftSpeed = scale*mDriverController.getRawAxis(1);
			rightSpeed = scale*mDriverController.getRawAxis(5);
			if (Math.abs(mDriverController.getRawAxis(5)) < .25 && Math.abs(mDriverController.getRawAxis(1)) < .4 && Math.abs(mDriverController.getRawAxis(0)) > .25) {
				leftSpeed = -scale*mDriverController.getRawAxis(0);
				rightSpeed = scale*mDriverController.getRawAxis(0);
			}
		}
	}
}
