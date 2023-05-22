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
  // START: Setup AdvantageKit
  public final class AdvantageKit {
    /** Set to true if you want to replay an existing AdvantageKit log back in simulation */
    static final boolean REPLAY_MODE = false;
  }
  // END: Setup AdvantageKit

  // START: Setup SysId
  public final class SysId {
    public final class Drive {
      public static final double maxSpeedMetersPerSecond = 0.65;
      public static final double maxRotationRadiansPerSecond = 18;

      public static final double combinedksVolts = 0.23565;
      public static final double combinedkvVoltSecondsPerMeter = 10.41;
      public static final double combinedkaVoltSecondsSquaredPerMeter = 0.81878;

      public static final double leftKsVolts = 0.15501;
      public static final double leftKvVoltSecondsPerMeter = 10.795;
      public static final double leftKaVoltSecondsSquaredPerMeter = 0.22663;

      public static final double leftKpVelocity = 0.71328;
      public static final double leftKiVelocity = 0;
      public static final double leftKdVelocity = 0;

      public static final double rightKsVolts = 0.27801;
      public static final double rightKvVoltSecondsPerMeter = 10.039;
      public static final double rightKaVoltSecondsSquaredPerMeter = 0.22158;

      public static final double rightKpVelocity = 0.78136;
      public static final double rightKiVelocity = 0;
      public static final double rightKdVelocity = 0;

      public static final double trackWidthMeters = 0.14692;
    }
    // END: Setup SysId
  }
}
