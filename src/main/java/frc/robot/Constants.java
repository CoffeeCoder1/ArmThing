// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double MANUAL_ARM_SLOW_FACTOR = 1.0/8.0;
  }
  public static class ArmConstants {
    public static final int PIVOT_MOTOR_CAN_ID = 1;
    public static final double GEAR_RATIO = 1.0/16.0;
    public static final double POSITION_CONVERSION_FACTOR = 360 * GEAR_RATIO;
    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0;
    public static final double ABS_POSITION_CONVERSION_FACTOR = 360;
    public static final double ABS_VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0;
    public static final double OFFSET = 307;
    public static final double KP = 0.004;//.005
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = 0;
    public static final double KG = 0.1;
    public static final double KV = 0;
    public static final double SETPOINT_TOLERANCE = 1;
  }
}
