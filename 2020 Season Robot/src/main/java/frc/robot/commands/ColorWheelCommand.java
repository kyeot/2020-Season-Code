/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheelSystem;
import frc.robot.subsystems.LEDSubSystem;

public class ColorWheelCommand extends CommandBase {

  private final ColorWheelSystem mColorWheelSubSystem;
  private final LEDSubSystem mLedSubSystem;
  private  String LastColor;
  private int bluecount;
  private int redcount;
  private int greencount;
  private int yellowcount;

  public ColorWheelCommand(ColorWheelSystem colorwheelsystem,LEDSubSystem ledsubsystem) {
    mColorWheelSubSystem = colorwheelsystem;
    mLedSubSystem = ledsubsystem;
    addRequirements(colorwheelsystem);
    //addRequirements(ledsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yellowcount = 0;
    bluecount = 0;
    redcount = 0;
    greencount = 0;
    LastColor = "";
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("ColorWheel Command","Active"  );
    mColorWheelSubSystem.TurnColorWheel();
    
  String Color = mColorWheelSubSystem.ReadColorSensor();
 if(Color =="Blue"){
   mLedSubSystem.SetBlueMode(); 
  }
  if(Color =="Red"){
    mLedSubSystem.SetRedMode();
  }
  if(Color=="Green"){
    mLedSubSystem.SetGreenMode();
  }
  if(Color=="Yellow"){
    mLedSubSystem.SetYellowMode();
  }
 
  if (LastColor != Color)
  {
    if(LastColor =="Blue"){
      bluecount=bluecount+1;
     
    }
    if(LastColor =="Red"){
      redcount=redcount+1;
    }
    if(LastColor =="Green"){
      greencount=greencount+1;
     
    }
    if(LastColor =="Yellow"){
      yellowcount=yellowcount+1;
    }

    LastColor = Color;
    
    SmartDashboard.putString("Last Color","" + LastColor  );
    SmartDashboard.putString("Green Count","" + greencount  );
    SmartDashboard.putString("Yellow Count","" + yellowcount  );
    SmartDashboard.putString("Blue Count","" + bluecount );
    SmartDashboard.putString("Red Count","" + redcount  );
  }  


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

      SmartDashboard.putString("ColorWheel Command","End"  );
      mColorWheelSubSystem.StopColorWheel();
      

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(yellowcount==3){
     return true;
    }
    if(redcount==3){
      return true;
    }
    if(bluecount==3){
      return true;
    }
    if(greencount==3){
      return true;
    }
    else{
      return false;
    }
   
  }
}
