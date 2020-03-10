

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveSubsystem;

import frc.robot.subsystems.LEDSubsystem;

import frc.robot.trajectories.SneakyTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CenterRight6Cell extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous.
   */
  public CenterRight6Cell(SneakyTrajectory s_trajectory, LEDSubsystem led) {
    addCommands(
        new RunCommand(led::red, led).withTimeout(3),
        s_trajectory.getRamsete(s_trajectory.centerRightAutoBackwards)
        .raceWith(new RunCommand(led::rainbow, led))
    );
  }
}