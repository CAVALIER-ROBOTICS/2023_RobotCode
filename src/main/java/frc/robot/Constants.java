// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

        public static final String[] fourBallAuto = {
                        "FourBallPath1",
                        "FourBallPath2"
        };

        public static final double WHEELRADMM = (3.9 * 2.54) / 2;
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
                        new Pose3d(0.0, .762, 0.0, new Rotation3d()));

        public static final String simpleAuto = "SimpleAuto";

        public static final int wristID = 44;
        public static final int wristRotID = 55;

        public static final int vacuum99ID = 99;
        public static final int vacuum88ID = 88;
        public static final int vacuum77ID = 77;

        public static final int armExtendID = 33;

        public static final int armAngle11ID = 11;
        public static final int armAngle22ID = 22;

        public final static double L = .5715;
        public final static double W = .5715;

        public static final PathConstraints CONSTRAINTS = new PathConstraints(4, 3);

        public final static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(L / 2, W / 2),
                        // Front right
                        new Translation2d(L / 2, -W / 2),
                        // Back left
                        new Translation2d(-L / 2, -W / 2),
                        // Back right
                        new Translation2d(-L / 2, W / 2));

        public static final int encoderCPR = 2048;
        // inches
        public static final double wheelDiamter = .1016;
        public static final double distancePerPulse =
                        (wheelDiamter * Math.PI) / (double) encoderCPR;

        public static double acceptedVolts = 65;

        public static double FIELDWIDTH = 26.0 + (7 / 12);
        public static double FIELDLENGTH = 57.0 + (1 / 12);

        // public static final TrapezoidProfile.Constraints thetaControllerConstraints =
        // new TrapezoidProfile.Constraints(maxAngularSpeed, maxAngularAcceleration);

        public static final class AutoConstants {
                public static final double maxSpeedMetersPerSecond = 4;
                public static final double maxAccelerationMetersPerSecondSquared = 3;

                public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
                public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;

                public static final double PIDXP = 0.0000007; 
                public static final double PIDYP = 0.00000;

                public static final double PIDXI = .000001;
                public static final double PIDXD = .0001;

                public static final double PIDYI = .000001;
                public static final double PIDYD = .0001;

                public static final double thetaP = .005; // .005
                public static final double thetaI = 0.0001;
                public static final double thetaD = 0;

                // Constraint for the motion profiled robot angle controller
                public static final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
                                maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);
                public static final double maxAccelerationMetersPerSecond = 0;
        }

}
