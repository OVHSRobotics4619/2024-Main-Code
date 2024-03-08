// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class Climber 
  {
    // Motor speeds on a scale of 0 to 1
    public static final double CLIMB_SPEED = 0.8;

    // Time that the climber climbs up (seconds)
    public static final double CLIMB_TIME = 1;
  }

  public static final class Shooter 
  {
    // Motor speeds on a scale of 0 to 1
    public static final double INTAKE_SPEED = 0.8;
    public static final double SHOOTING_SPEED = 1;

    // Time (seconds) for how long the robot intakes (all 4 wheels)
    static final public double INTAKE_TIME = 1;

    // Time (seconds) for how long the shooter charges (front 2 wheels only)
    static final public double SHOOTING_RAMPUP_TIME = 1;

    // Time (seconds) for how long the shooter shoots after charging (all 4 wheels)
    static final public double SHOOTING_SHOOT_TIME = 1;


  }

  public static final class OIConstants {
    // http://team358.org/files/programming/ControlSystem2009-/XBoxControlMapping.jpg

    public static final int XBOX_PORT = 0;
    
    // Xbox controller button mappings
    public final static int A = 1;
    public final static int B = 2;
    public final static int X = 3;
    public final static int Y = 4;
    public final static int L_BUMPER = 5;
    public final static int R_BUMPER = 6;
    public final static int BACK = 7;
    public final static int START = 8;	
    public final static int LEFT_STICK_PRESS = 9;	
    public final static int RIGHT_STICK_PRESS = 10;	
    
    // Xbox controller axis mappings
    public final static int LEFT_STICK_X_AXIS = 0;
    public final static int LEFT_STICK_Y_AXIS = 1;
    public final static int LEFT_TRIGGER = 2;
    public final static int RIGHT_TRIGGER = 3;
    public final static int RIGHT_STICK_X_AXIS = 4;
    public final static int RIGHT_STICK_Y_AXIS = 5;
  }

  public static class AprilTags
  {

    // Multiplier for how fast the robot turns towards the apriltag
    public static final double TURN_SPEED_MULTIPLIER = 5;

    // How close the robot turns to the apriltag before it stops (degrees)
    public static final double TURN_STOP_LIMIT = 1;

    public static final class CameraConstants {
      public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
      public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(24);
      public static final double CAMERA_PITCH_RADIANS = Units.inchesToMeters(24);
      public static final double GOAL_RANGE_METERS = Units.inchesToMeters(24);
    }
  }
}
