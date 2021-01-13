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
        public Trajectory centerRightAutoBackwards, centerRightAutoForward, uTurn, rightTurn, sTurnForward, sTurnBackward, barrelRace;
        //public Trajectory centerRightAutoBackwards, centerRightAutoForward, uTurn, rightTurn, sTurnForward, sTurnBackward, barrelRace;
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

                barrelRace = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                    new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                    // Pass through these two interior waypoints, making an 'barrel race' curve path
                    List.of(
                        new Translation2d( 0.7704, 0.1183),//2
                        new Translation2d(1.8718, 0.2336),//3
                        new Translation2d(2.5214, 0.2348),//4
                        new Translation2d(3.3253, -0.3257),//5
                        new Translation2d(3.3507, -1.1316),//6
                        new Translation2d(2.7556, -1.4749),//7
                        new Translation2d(2.1664, -1.2472),//8
                        new Translation2d(2.0189, -0.7286),//9
                        new Translation2d(2.2964, -0.2003),//10
                        new Translation2d(2.9616, 0.0707),//11
                        new Translation2d(3.6843, -0.0038),//12
                        new Translation2d(4.8717, -0.2473),//13
                        new Translation2d(5.7311, 0.0360),//14
                        new Translation2d(6.0631, 0.7137),//15
                        new Translation2d(5.9352, 1.2199),//16
                        new Translation2d(4.7980, 1.4766),//17
                        new Translation2d(4.2212, 0.8637),//18
                        new Translation2d(4.3007, -0.0741),//19
                        new Translation2d(5.0506, -0.9636),//20
                        new Translation2d(6.1040, -1.4034),//21
                        new Translation2d(6.7567, -1.3331),//22
                        new Translation2d(7.1125, -1.0022),//23
                        new Translation2d(7.1316, -0.5690),//24
                        new Translation2d(6.7634, -0.0224),//25
                        new Translation2d(6.2146, 0.1709),//26
                        new Translation2d(5.2259, 0.1906),//27
                        new Translation2d(3.0449, 0.1953)//28
                    // new Translation2d(2, -1)
                     ),
                //     // End 3 meters straight ahead of where we started, facing forward
                     new Pose2d(0.5, 0.1953, Rotation2d.fromDegrees(170)),
                     //new Pose2d(2.0189, -0.7286, Rotation2d.fromDegrees(3.1)),
                     // Pass config
                     configForward);                
                

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