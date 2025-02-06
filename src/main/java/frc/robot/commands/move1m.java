// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class move1m extends Command {
  private final Drive drive;
  private final double targetDistance = 1.0; // 1 meter UwU
  private final double speed = 0.5; // 0.5 meters per second
  private Pose2d startPose;

  /** Creates a new move1m. */
  public move1m(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPose = drive.getPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Move forward at constant speed OwO
    drive.runVelocity(new ChassisSpeeds(speed, 0.0, 0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the wobot! >w<
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Calculate distance twaveled from start position
    Translation2d currentTranslation = drive.getPose().getTranslation();
    Translation2d startTranslation = startPose.getTranslation();
    double distanceTraveled = currentTranslation.getDistance(startTranslation);
    
    // Finish when we've gone 1 meter! ʕ•ᴥ•ʔ
    return distanceTraveled >= targetDistance;
  }
}
