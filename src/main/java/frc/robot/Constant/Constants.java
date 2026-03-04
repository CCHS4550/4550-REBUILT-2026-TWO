// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.Constant;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Globals;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class SysIdConstants {
    public static final Velocity<VoltageUnit> TRANSLATION_RAMP_RATE = null;
    public static final Voltage TRANSLATION_STEP_RATE = Units.Volts.of(7);
    public static final Time TRANSLATION_TIMEOUT = Units.Seconds.of(5);

    /* This is in radians per second², but SysId only supports "volts per second" */
    public static final Velocity<VoltageUnit> ROTATION_RAMP_RATE =
        Units.Volts.of(Math.PI / 6).per(Units.Second);
    /* This is in radians per second, but SysId only supports "volts" */
    public static final Voltage ROTATION_STEP_RATE = Units.Volts.of(Math.PI);
    public static final Time ROTATION_TIMEOUT = Units.Seconds.of(5);

    public static final Velocity<VoltageUnit> STEER_RAMP_RATE = null;
    public static final Voltage STEER_STEP_RATE = Units.Volts.of(7);
    public static final Time STEER_TIMEOUT = null;

    public static final double DRIVE_GEAR_RATIO = 6.38;
  }

  public static final class TurretConstants {

    public static final double HOOD_PRECISION = (2.0) * ((2 * Math.PI) / 360.0);
    public static final double FLYWHEEL_RPM_PRECISION = 200.0;
    public static final double HOOD_FEED_PRECISION = (5.0) * ((2 * Math.PI) / 360.0);
    public static final double FLYWHEEL_RPM_FEED_PRECISION = 400.0;
    public static final double ELEVATION_GEAR_RATIO = 1.0 / 149.08; // fill
    public static final double ELEVATION_ENCODER_GEAR_RATIO = 1.0 / 8.80;
    public static final double ELEVATION_POSITION_COEFFICIENT = 2 * Math.PI * ELEVATION_GEAR_RATIO;

    public static final double ELEVATION_ENCODER_POSITION_COEFFICIENT =
        2 * Math.PI * ELEVATION_ENCODER_GEAR_RATIO;

    public static final double SHOT_DEBOUNCE_S = 0.08;
    public static final double SHOT_ACCEL_LOW = -30.0;
    public static final double SHOT_ACCEL_HIGH = 36.0;
    public static final double SHOT_RPM_DELTA_LOW = -180.0;
    public static final double SHOT_RPM_DELTA_HIGH = 200.0;
    public static final double SHOT_SPIKE_CURRENT = 27.0;

    public static final double ROTATION_GEAR_RATIO = 1.0 / 37;
    public static final double ROTATION_GEAR_RATIO_TO_ENCODER = 1.0 / 7.41;
    public static final double ROTATION_POSITION_COEFFICIENT = 2 * Math.PI * ROTATION_GEAR_RATIO;
    public static final double ROTATION_POSITION_COEFFICIENT_TO_ENCODER =
        2 * Math.PI * ROTATION_GEAR_RATIO_TO_ENCODER;

    public static final Rotation2d ROTATION_MINANGLE = Rotation2d.fromDegrees(-180);
    public static final Rotation2d ROTATION_MAXANGLE = Rotation2d.fromDegrees(180);

    // public static final double ELEVATION_DEFAULT_ENCODER_READING_AT_SHALLOWEST_ANGLE = -0.48;
    public static final double ELEVATION_DEFAULT_ENCODER_READING_AT_SHALLOWEST_ANGLE = 0.35;
    public static final double SHALLOWEST_POSSIBLE_ELEVATION_ANGLE_RADIANS = Math.toRadians(48.0);
    public static final double STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS = Math.toRadians(75);

    public static final double SHOOTER_MAX_RADIANS_PER_SEC = 500.0;

    public static final double TURRET_HEIGHT_METERS = 14.523750;

    public static final Transform2d TURRET_TRANSFORM =
        new Transform2d(-0.040, 2.044, new Rotation2d(0));

    public static final Translation3d TURRET_TRANSFORM_3D =
        new Translation3d(-0.040, 2.044, 14.523570);

    public static final double MAX_BALL_HEIGHT_METERS = 5.4864;

    public static final double FORWARD_ROTATION_LIMIT_RADIANS = 0.0;

    public static final double BACKWARDS_ROTATION_LIMIT_RADIANS = 0.0;

    private static final double DISTANCE_OFFSET = 0.0;
    private static final double ANGLE_OFFSET = 0.0;
    private static final double RPM_OFFSET = 0.0;
    private static final double TOF_OFFSET = 0.0;
    // Distance in meters, Hood Angle, Flywheel RPM, Time of Flight in seconds
    public static final double[][] SHOT_MAP =
        new double[][] {
          {1.45, 85, 1690, 1.07},
          {1.8, 82, 1850, 1.11},
          {2.17, 80.44, 1950, 1.17},
          {2.48, 80.44, 2000, 1.32},
          {2.76, 80.44, 2150, 1.35},
          {3.34, 78, 2250, 1.34},
          {3.67, 75, 2300, 1.35},
          {3.99, 75, 2400, 1.42},
          {4.46, 73, 2400, 1.35},
          {4.8, 70, 2400, 1.33},
          {5.22, 70, 2550, 1.36},
          {5.53, 65, 2550, 1.35},
          {6.11, 60, 2700, 1.27},
          {6.55, 60, 2800, 1.35},
          {6.7, 60, 2900, 1.33},
        };

    private static final double FEED_DISTANCE_OFFSET = 0.0;
    private static final double FEED_ANGLE_OFFSET = 0.0;
    private static final double FEED_RPM_OFFSET = 0.0;
    private static final double FEED_TOF_OFFSET = 0.0;
    // Distance in meters, Hood Angle, Flywheel RPM, Time of Flight in seconds
    public static final double[][] FEED_SHOT_MAP =
        new double[][] {
          {1, 60, 700, 0.74},
          {1.5, 60, 900, 0.85},
          {1.79, 60, 1254, 0.84},
          {2.5, 60, 1400, 1.02},
          {3.01, 60, 1600, 1.0},
          {3.2, 60, 1750, 1.0},
          {3.49, 60, 1800, 1.01},
          {4.04, 60, 1900, 1.14},
          {4.48, 60, 2000, 1.21},
          {5.01, 60, 2150, 1.15},
          {5.52, 60, 2250, 1.35},
          {5.99, 60, 2350, 1.44},
          {6.42, 60, 2550, 1.38},
          {6.99, 60, 2700, 1.52},
          {7.51, 60, 2850, 1.57},
        };

    static {
      for (int i = 0; i < SHOT_MAP.length; i++) {
        SHOT_MAP[i][0] += DISTANCE_OFFSET;
        SHOT_MAP[i][1] += ANGLE_OFFSET;
        SHOT_MAP[i][2] += RPM_OFFSET;
        SHOT_MAP[i][3] += TOF_OFFSET;
      }

      for (int i = 0; i < FEED_SHOT_MAP.length; i++) {
        FEED_SHOT_MAP[i][0] += FEED_DISTANCE_OFFSET;
        FEED_SHOT_MAP[i][1] += FEED_ANGLE_OFFSET;
        FEED_SHOT_MAP[i][2] += FEED_RPM_OFFSET;
        FEED_SHOT_MAP[i][3] += FEED_TOF_OFFSET;
      }
    }
  }

  public static final class IntakeConstants {
    public static final double EXTENSION_GEAR_RATIO = 1.0 / 25.0; // fill
    public static final double EXTENSION_POSITION_COEFFICIENT = 2 * Math.PI * EXTENSION_GEAR_RATIO;
  }

  public static final class LowerChassisConstants {
    public static final double KICKER_GEAR_RATIO =
        (60.0 / 12.0) * (60.0 / 16.0) * (58.0 / 9.0); // fill
    public static final double KICKER_POSITION_COEFFICIENT = 2 * Math.PI * KICKER_GEAR_RATIO;
  }

  public static final class VisionConstants {
    public static final double linearStdDevBaseline = 0.02;
    public static final double angularStdDevBaseline = 0.06;
    public static final double maxAmbiguity = 0.01;
    public static final double maxZError = 6.75;
  }

  public static final class ShooterCalculationConstants {
    public static final double GRAVITATION_CONSTANT = 9.81;
    public static final double TIME_DELAY = 0.03;
    public static final double GEOMETRY_VELOCITY = 6.7;
    public static final double TURRET_HEIGHT = 0.12;
    // public static final Pose2d HUB_POSITION = new Pose2d(4.625594,4.034663,new Rotation2d());

  }

  public static final class Field {
    public static final double FIELD_WIDTH = 316.64 / 39.37;
    public static final double FIELD_LENGTH = 650.12 / 39.37;
    public static final double BLUE_HUB_X = 182.1 / 39.37;
    public static final double RED_HUB_X = FIELD_LENGTH - BLUE_HUB_X;
    public static final double HUB_Y = FIELD_WIDTH / 2;
    public static final double HUB_Z = 1.83;
    public static final Translation3d HUB_POSE_BLUE = new Translation3d(BLUE_HUB_X, HUB_Y, HUB_Z);
    public static final Translation3d HUB_POSE_RED = new Translation3d(RED_HUB_X, HUB_Y, HUB_Z);
    public static final double BUMP_WIDTH = 44.4 / 39.37;

    public static final Translation2d RED_LEFT_FEED_POSE = new Translation2d(13.0, 1.85);
    public static final Translation2d RED_RIGHT_FEED_POSE =
        new Translation2d(RED_LEFT_FEED_POSE.getX(), FIELD_WIDTH - RED_LEFT_FEED_POSE.getY());
    public static final Translation2d BLUE_LEFT_FEED_POSE =
        new Translation2d(FIELD_LENGTH - RED_LEFT_FEED_POSE.getX(), RED_RIGHT_FEED_POSE.getY());
    public static final Translation2d BLUE_RIGHT_FEED_POSE =
        new Translation2d(BLUE_LEFT_FEED_POSE.getX(), RED_LEFT_FEED_POSE.getY());

    public static Translation3d getHubPose() {
      if (Globals.fieldSide.equals("blue")) {
        return HUB_POSE_BLUE;
      } else {
        return HUB_POSE_RED;
      }
    }

    public static Translation2d getFeedTarget(Translation2d turretPose) {
      if (Globals.fieldSide.equals("blue")) {
        if (turretPose.getY() > FIELD_WIDTH / 2) {
          return BLUE_LEFT_FEED_POSE;
        } else {
          return BLUE_RIGHT_FEED_POSE;
        }
      } else {
        if (turretPose.getY() < FIELD_WIDTH / 2) {
          return RED_LEFT_FEED_POSE;
        } else {
          return RED_RIGHT_FEED_POSE;
        }
      }
    }

    public static boolean isInAllianceZone(Translation2d robotPosition) {
      if (Globals.fieldSide.equals("red")) {
        return HUB_POSE_RED.getX() + BUMP_WIDTH / 2 < robotPosition.getX();
      } else {
        return robotPosition.getX() < HUB_POSE_BLUE.getX() - BUMP_WIDTH / 2;
      }
    }

    public static boolean isOnBump(Translation2d robotPosition) {
      boolean onBlueBump = Math.abs(robotPosition.getX() - HUB_POSE_BLUE.getX()) < BUMP_WIDTH / 2;
      boolean onRedBump = Math.abs(robotPosition.getX() - HUB_POSE_RED.getX()) < BUMP_WIDTH / 2;
      return onBlueBump || onRedBump;
    }

    public static final double HUB_RADIUS = 21.0 / 39.37;
    public static final double FEED_RADIUS = 33.39 / 39.37;
    public static final double BALL_WIDTH = 0.15;
  }
}
