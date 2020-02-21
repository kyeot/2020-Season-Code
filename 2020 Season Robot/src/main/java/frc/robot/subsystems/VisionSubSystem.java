/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.text.DecimalFormat;
import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.vision.VisionRunner;
//import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionPipeline;
import frc.robot.Constants;
import frc.robot.util.GripPipelineBlue;

public class VisionSubsystem extends SubsystemBase {
  /**
   * Creates a new VisionSubSystem.
   */

  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 240;

  private VisionThread visionThread;
  private Thread thread;
  private MjpegServer server;
  private double centerX = 0.0;

  private final Object imgLock = new Object();

  private MatOfPoint mop;
  private Rect r;

  private UsbCamera camera;
  private CvSink cvSink;
  private CvSource outputStream;

  private double currentFrameTime;
  private double prevFrameTime;

  public VisionSubsystem() {

    camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    cvSink = CameraServer.getInstance().getVideo();
    cvSink.setEnabled(true);
    cvSink.setSource(camera);

    outputStream = CameraServer.getInstance().putVideo("BoundingRectTest", 320, 240);
    server = new MjpegServer("OutputServer", 2783);
    server.setSource(outputStream);

    //pipeline = new GripPipelineIV();
    startThreads();
    SmartDashboard.putString("DB/String 0", visionThread.getState().toString());
  }

  public void startThreads() {

    
    visionThread = new VisionThread(camera, new GripPipelineBlue(), pipeline -> {
      //Mat image = new Mat();
      if (!pipeline.findContoursOutput().isEmpty()) {
        
        // loop through array and find left and right bounds
        //if (pipeline.findContoursOutput().size() == 1) {
          //mop = pipeline.findContoursOutput().get(pipeline.findContoursOutput().size() - 1);
          mop = pipeline.findContoursOutput().get(0);
          r = Imgproc.boundingRect(mop);
          synchronized (imgLock) {
            // centerX = 100; // r.x + (r.width / 2);
            centerX = r.x + (r.width / 2);
          }
        
      }
  
      //DEBUG CODE
      // if (pipeline.findContoursOutput().size() > 1) {
      //   mop = pipeline.findContoursOutput().get(0);
      //   r = Imgproc.boundingRect(mop);
      //   centerX = r.x + (r.width / 2);
      // } else {
      //   centerX = 0;
      // }
      
      //outputStream.putFrame(image);
      // if (!pipeline.filterContoursOutput().isEmpty()) {
      // Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));

      // }
    });

    visionThread.start();

    // new Thread(() -> {
    //   UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    //   camera.setResolution(640, 480);

    //   CvSink cvSink = CameraServer.getInstance().getVideo();
    //   CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

    //   Mat source = new Mat();
    //   Mat output = new Mat();
    // }).start();
    //   int counter = 0;
    //   while(!Thread.interrupted()) {
    //     if (cvSink.grabFrame(source) == 0) {
    //       continue;
    //     }
    //     synchronized(imgLock) {
    //       SmartDashboard.putString("DB/String 5", "" + "Here");
    //       Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
    //       //Imgproc.rectangle(output, r.tl(), r.br(), new Scalar(0, 255, 255), 2);
    //       outputStream.putFrame(output);
    //       System.out.println("Top left: " + r.tl());
    //       counter++;
    //       SmartDashboard.putString("DB/String 5", "" + counter);
      
    //     }
    //   }
    // }).start();
  }

  public double getCenterX() {
    return centerX;
  }


  public double getRawAngle() {

    return (getCenterX() - (GripPipelineBlue.PIPELINE_WIDTH / 2)) * (85.0 / GripPipelineBlue.PIPELINE_WIDTH);
  }

  DecimalFormat df = new DecimalFormat("#.##");
  public String getDistance() {
    double distance = (Constants.kGoalTargetSize * GripPipelineBlue.PIPELINE_WIDTH)/(2 * r.width * Math.tan(degreesToRadians(46.5)));
    //SmartDashboard.putString("DB/String 9", "Debug: " + df);
    return df.format(distance);
  }

  public double degreesToRadians(double degrees) {
    
    return degrees * 0.017453292519943295;
  }

  // public double getRobotToTarget() {

  // }

  public Rect getBoundingRect() {
    if (r != null) {
      return r;
    } else {
      return new Rect(0, 0, 0, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}