/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Spark;


public class LEDSubSystem extends SubsystemBase {


  Spark ledController;

  public LEDSubSystem() {
    ledController = new Spark(0);

  }

  public void SetLEDMode(double speed) {
    //below logic tree allows robot to pull values from right motor
    //the led the correlated the motor output to a single color value for forward, reverse, and zero

    if(speed >= .01)
    {
      ledController.setSpeed(0.87);
    //sets "forward" color
    } 
    if(speed <= -.01)
    {
      ledController.setSpeed(0.61);
    //sets "reverse" color
    }
    if(speed < .01 && speed > -.01)
    {
      ledController.setSpeed(0.99);
    //sets "standby" color after the "x" button is pressed
    }
    
     
     
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
