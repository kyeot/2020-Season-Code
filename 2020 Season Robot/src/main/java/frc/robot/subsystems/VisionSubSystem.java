/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.vision.VisionRunner;
//import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;
import frc.robot.util.GripPipeline;


public class VisionSubSystem extends SubsystemBase {
  /**
   * Creates a new VisionSubSystem.
   */


  private static final int IMG_WIDTH = 160;
  private static final int IMG_HEIGHT = 120;

  private VisionThread visionThread;
  private double centerX = 0.0;

  private final Object imgLock = new Object();

  public VisionSubSystem() {

    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {

        if (!pipeline.findContoursOutput().isEmpty()) {
          //loop through array and find left and right bounds

          synchronized (imgLock) {
            centerX =  100; // r.x + (r.width / 2);
        }
        }
             
        //if (!pipeline.filterContoursOutput().isEmpty()) {
            //Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));

        //}
    });

    visionThread.start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
