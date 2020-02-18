/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Constants {

    public static final int kDriveController = 0;



    public static final double kGyroMaxAge = 0.6;   
    public static final double kRobotFront = 180;

    public static final int kIntakeMotorPort = 2;
    public static final double kIntakeMotorSpeed = 0.8;

    public static final int kColorWheelMotorPort = 21;
    public static final double kColorWheelMotorSpeed = 0.5;

    public static final int kLiftMotorPort = 20;
    public static final double kLiftMotorSpeed = 0.7;

    public static final int kImageWidth = 160;
    public static final int kImageHeight = 120;


    public static final double robotToCameraAngle = 0;

    public static final double kGoalTargetSize = 39.25; //inches

    public static final class DriveConstants {
       // competition settings
       
       /*
        public static final int kLeftMotor1Port = 12;
        public static final int kLeftMotor2Port = 13;
        public static final int kRightMotor1Port = 14;
        public static final int kRightMotor2Port = 15;
        */

        public static final int kLeftMotor1Port = 11;
        public static final int kLeftMotor2Port = 12;
        public static final int kRightMotor1Port = 13;
        public static final int kRightMotor2Port = 15;

        public static final int kUltrasonicPort = 0;
        public static double kValueToInches = 0.125;
    
        public static final int[] kLeftEncoderPorts = new int[]{0, 1};
        public static final int[] kRightEncoderPorts = new int[]{2, 3};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;
    
        //Using Feet to start
        //public final double kEncoderPulsePerFoot = 21.75;
        //public static final double kEncoderDistancePerPulse = 0.045977;

        public static final double kEncoderFootPerDistance = 21.75;
        public static final double kAutonomousDriveSpeed = 0.2;

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;


        public static final boolean kGyroReversed = false;

        public static final double kStabilizationP = 1;
        public static final double kStabilizationI = 0.5;
        public static final double kStabilizationD = 0;
    
        public static final double kTurnP = 1;
        public static final double kTurnI = 0;
        public static final double kTurnD = 0;
    
        public static final double kMaxTurnRateDegPerS = 100;
        public static final double kMaxTurnAccelerationDegPerSSquared = 300;
    
        public static final double kTurnToleranceDeg = 5;
        public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

        public static final int kBackwardsDrive = 1; 
      }

      public static final class ShooterConstants {
        public static final int kShooterMotorPort = 1;
      }



}
