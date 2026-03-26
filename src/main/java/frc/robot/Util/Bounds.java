// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Bounds {
  double minX;
  double maxX;
  double minY;
  double maxY;

  public Bounds(double minX, double maxX, double minY, double maxY) {
    this.minX = minX;
    this.maxX = maxX;
    this.minY = minY;
    this.maxY = maxY;
  }

  public Bounds(Pose2d leftCorner, Pose2d rightCorner) {
    minX = Math.min(leftCorner.getTranslation().getX(), rightCorner.getTranslation().getX());
    maxX = Math.max(leftCorner.getTranslation().getX(), rightCorner.getTranslation().getX());
    minY = Math.min(leftCorner.getTranslation().getY(), rightCorner.getTranslation().getY());
    maxY = Math.max(leftCorner.getTranslation().getY(), rightCorner.getTranslation().getY());
  }

  /** Whether the translation is contained within the bounds. */
  public boolean contains(Translation2d translation) {
    return translation.getX() >= minX
        && translation.getX() <= maxX
        && translation.getY() >= minY
        && translation.getY() <= maxY;
  }

  /** Clamps the translation to the bounds. */
  public Translation2d clamp(Translation2d translation) {
    return new Translation2d(
        MathUtil.clamp(translation.getX(), minX, maxX),
        MathUtil.clamp(translation.getY(), minY, maxY));
  }
}
