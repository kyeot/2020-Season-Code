/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShooterSubSystem extends SubsystemBase {

  //VictorSPX mShooterMotor;
  private CANSparkMax mShooterMotor;

  public ShooterSubSystem() {

   // mShooterMotor = new VictorSPX(ShooterConstants.kShooterMotorPort);

    mShooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);


    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
   // mShooterMotor.restoreFactoryDefaults();
    SmartDashboard.putString("SPark ","" + mShooterMotor.getOutputCurrent() );
  }

  public void SetShooterSpeed(double speed) {
    SmartDashboard.putString("SPark ","" + mShooterMotor.getOutputCurrent() );
    mShooterMotor.set(speed);   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
