/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubSystem;
import frc.robot.subsystems.LiftSubSystem;
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
  private final XboxController mManipulatorController;
  private final LEDSubSystem mLEDSubsystem;
  private final LiftSubSystem mLiftSubSystem;

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
  public DriveCommand( DriveSubsystem drivesubsystem, XboxController drivercontroller,XboxController manipulatorcontroller, LEDSubSystem ledsubsystem,LiftSubSystem liftsubsystem) {
    mDriveSubSystem = drivesubsystem;
	mDriverController = drivercontroller;
	mManipulatorController = manipulatorcontroller;
	mLEDSubsystem = ledsubsystem;
	mLiftSubSystem = liftsubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
	addRequirements(drivesubsystem);
	addRequirements(ledsubsystem);
	addRequirements(liftsubsystem);

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


	double distanceToTarget = mDriveSubSystem.GetSensorDistanceInInches();

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

	if(mDriverController.getRawButton(DriveConstants.kBackwardsDrive) == true && lastButton1State == false) {
		reverseButton1Toggle = toggleInput(reverseButton1Toggle);
		lastButton1State = true;
	} else if (mDriverController.getRawButton(DriveConstants.kBackwardsDrive) == false) {
		lastButton1State = false;
	}
	
	//if(reverseButton1Toggle) {
		mDriveSubSystem.SetLeftDriveSpeed(-leftSpeed);
		mDriveSubSystem.SetRightDriveSpeed(-rightSpeed);
	//}
	//else{
	//	mDriveSubSystem.SetLeftDriveSpeed(leftSpeed);
	//	mDriveSubSystem.SetRightDriveSpeed(rightSpeed);
	//}

	double averageSpeed = (Math.abs(leftSpeed) + Math.abs(rightSpeed)) /2;

	if (averageSpeed == 0) {
		mLEDSubsystem.SetRestingMode();
	}

	if (averageSpeed  >0 && averageSpeed < 0.3) {
		mLEDSubsystem.SetDrivingSlowMode();
	}

	if (averageSpeed  >=0.3 && averageSpeed < 0.7) {
		mLEDSubsystem.SetDrivingMediumMode();
	}

	if (averageSpeed  >=0.7 ) {
		mLEDSubsystem.SetDrivingFastMode();
	}

    // Manual Lift - Need to change controllers
	if ( mManipulatorController.getPOV()  == 0 ) {
        mLiftSubSystem.RaiseLift();
    } else if (mManipulatorController.getPOV() == 180) {
        mLiftSubSystem.LowerLift();
    } else {
        mLiftSubSystem.StopLift();
    }
	
	//SmartDashboard.putString("left","" + speedLeft );
    SmartDashboard.putString("reverseButton1Toggle","" + reverseButton1Toggle);
    SmartDashboard.putString("l Distance","" + mDriveSubSystem.getLeftEncoderDistance() );
    SmartDashboard.putString("r Distance","" + mDriveSubSystem.getRightEncoderDistance() );
	SmartDashboard.putString("Heading: ","" + mDriveSubSystem.getHeading());
	SmartDashboard.putString("Distance to Target: ","" + distanceToTarget);
	SmartDashboard.putString("POV2: ","" + mDriverController.getPOV()) ;
		
  }

  public boolean toggleInput(boolean value) {
	return value ? false : true;
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
			leftSpeed = -scale*mDriverController.getRawAxis(5);
			rightSpeed = -scale*mDriverController.getRawAxis(1);
			if (Math.abs(mDriverController.getRawAxis(1)) < .25 && Math.abs(mDriverController.getRawAxis(5)) < .4 && Math.abs(mDriverController.getRawAxis(0)) > .25) {
				leftSpeed = -scale*mDriverController.getRawAxis(0);
				rightSpeed = scale*mDriverController.getRawAxis(0);
			}
		}
	}
}
