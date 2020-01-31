/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    public static final int CAN_ID_LEFT_DRIVE = 1;
    public static final int CAN_ID_RIGHT_DRIVE = 2;
    public static final int CAN_ID_LEFT_DRIVE_2 = 3;
    public static final int CAN_ID_RIGHT_DRIVE_2 = 4;

    public static final int CAN_ID_BALL_LAUNCH_LEFT = 6;
    public static final int CAN_ID_BALL_LAUNCH_RIGHT = 20;

    public static final int[] kLeftEncoderPorts = new int[]{0, 1};
    public static final int[] kRightEncoderPorts = new int[]{2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterInches = 6;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;
    public static final double kSlewSpeed = 36;
    public static final double kSlewTurn = 48;
  }

  public static final class BallLauncherConstants {
    public static final double kLaunchspeed = 4000; //in rpm
    
  }
  public static final class HopperConstants {
    public static final int KEL_LIMIT_SWITCH = 1; //upper limit switch
    public static final int GUS_LIMIT_SWITCH = 2; //lower limit switch
    public static final int CAN_ID_Hopper_Axle = 21; //Axle
  }
  public static final class IntakeConstants {
    
   
    public static final double intakespeed = 0.2; //speed
    public static final int CAN_ID_Hopper_Intake = 22; //Intake
  }
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }
}