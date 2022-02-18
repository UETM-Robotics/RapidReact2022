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

    public static final class PortConstants {
        public static final int frontLeftPort = 1;
        public static final int hindLeftPort = 2;

        public static final int frontRightPort = 3;
        public static final int hindRightPort = 4;

        public static final int shooterMotorPort = 5;
        public static final int beltTransporterMotorPort = 6;

        public static final int intakeMotorPort = 7;
    }

    public static final class TrajectoryConstants {

        public static final double velocityConversionFactorMeters = 0.000745064924;
        public static final double positionConversionFactorMeters = 0.04470389546;
        public static final double robotTrackWidthInches = 22.25;

        public static final boolean useRotationalPID = false;
        public static final boolean useVelocityPID = false;

        public static final double robotMaxVelocityMPS = 4.2;

    }
}
