/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Talon;

public class LEDSubSystem extends SubsystemBase {


  Talon ledController;

  public LEDSubSystem() {
    ledController = new Talon(0);
  }

  public void SetLEDMode(double speed) {
    //rightMotor.set(ControlMode.PercentOutput, speed);
    //LEDPort2.setAngle(30);
    ledController.setSpeed(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
