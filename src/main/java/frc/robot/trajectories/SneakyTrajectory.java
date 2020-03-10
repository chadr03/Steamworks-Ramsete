/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Add your docs here.
 */
public class SneakyTrajectory {
        public Trajectory centerRightAutoBackwards, centerRightAutoForward, uTurn, rightTurn, sTurnForward, sTurnBackward;
       
        private DriveSubsystem m_drive;


        public SneakyTrajectory(DriveSubsystem drive) {
                m_drive = drive;

                var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics, 10); // 8

                TrajectoryConfig configReversed = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                                DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveConstants.kDriveKinematics)
                                                // Apply the voltage constraint
                                                .addConstraint(autoVoltageConstraint);
                configReversed.setReversed(true);

                TrajectoryConfig configForward = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                                DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveConstants.kDriveKinematics)
                                                // Apply the voltage constraint
                                                .addConstraint(autoVoltageConstraint);
                double divisor = 1.0;
                centerRightAutoBackwards = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                            new Pose2d(0, 0, new Rotation2d(0)),
                            // Pass through these two interior waypoints, making an 's' curve path
                            List.of(new Translation2d(0.5, -0.25)
                            // new Translation2d(2, -1)
                            ),
                            // End 3 meters straight ahead of where we started, facing forward
                            new Pose2d(1, -0.5, new Rotation2d(0)),
                            // Pass config
                            configForward);

                             
                centerRightAutoForward = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(8 / divisor, -7.55 / divisor, new Rotation2d(0)),
                                                new Pose2d(10.5 / divisor, -5.80 / divisor, new Rotation2d(0)),
                                                new Pose2d(13 / divisor, -5.80 / divisor, new Rotation2d(0))),
                                configForward);
                
                uTurn = TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                    
                    List.of(new Translation2d(0.5, -0.25)
                    
                    ),
                    
                    new Pose2d(0, -0.5, Rotation2d.fromDegrees(180)),
                    // Pass config
                    configForward);

                rightTurn = TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                
                    List.of(new Translation2d(0.5, -0.1)
                    
                    ),
                
                    new Pose2d(1, -0.5, Rotation2d.fromDegrees(-90)),
                    // Pass config
                    configForward);
                
                sTurnForward = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                    new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(0.5, -0.25)
                    // new Translation2d(2, -1)
                    ),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(1, -0.5, Rotation2d.fromDegrees(0)),
                    // Pass config
                    configForward);
                
                sTurnBackward = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                    new Pose2d(1, -0.5, Rotation2d.fromDegrees(0)),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(0.5, -0.25)
                    // new Translation2d(2, -1)
                    ),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                    // Pass config
                    configReversed);
                

        }

        public RamseteCommand getRamsete(Trajectory traj) {
                return new RamseteCommand(traj, m_drive::getPose,
                                new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds,
                                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                new PIDController(DriveConstants.kPDriveVel, 0, 0), m_drive::tankDriveVolts, m_drive);
        }
}