

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
public class FakeAutoCommand extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous.
   */
  public FakeAutoCommand(SneakyTrajectory s_trajectory, LEDSubsystem led, DriveSubsystem m_drive) {
    addCommands(
        //auto aiming with limelight
        new RunCommand(led::green, led).withTimeout(3),
        //run a trajectory to go get something and make the leds rainbow at the same time
        s_trajectory.getRamsete(s_trajectory.sTurnForward)
        .raceWith(new RunCommand(led::rainbow, led)),
        //then do the same trajectory backwards
        s_trajectory.getRamsete(s_trajectory.sTurnBackward)
        .raceWith(new RunCommand(led::bluePulse, led)),
        new RunCommand(()->m_drive.tankDriveVolts(0, 0),m_drive)
        .raceWith(new RunCommand(led::red, led))
    );
  }
}