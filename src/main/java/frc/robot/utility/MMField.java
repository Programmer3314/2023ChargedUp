// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class MMField {
    private static double cellSpacing = 0.56;
    private static double redRefCell = -2.93 - (cellSpacing * 2);
    private static double blueRefCell = 0.416 + (cellSpacing * 2);

    public static Pose2d getCellPose(int cell, boolean isRedAlliance) {
        double y;
        double x;
        double r;
        if (isRedAlliance) {
            y = redRefCell + cell * cellSpacing;
            x = 6;
            r = 0;
        } else {
            y = blueRefCell - cell * cellSpacing;
            x = -6;
            r = 180;
        }

        return new Pose2d(x, y, new Rotation2d(Math.toRadians(r)));
    }

    public static boolean isCellCone(int cell) {
        return cell == 1 || cell == 3 || cell == 4 || cell == 6 || cell == 7 || cell == 9;
    }
}
