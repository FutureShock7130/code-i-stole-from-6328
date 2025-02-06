package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class TurnToAngle extends Command {
  private final Drive drive;
  private final Rotation2d targetAngle;
  private final double rotationSpeed = 0.5; // Rotation speed in radians per second
  private final double angleTolerance = 0.05; // About 3 degrees of tolerance UwU

  public TurnToAngle(Drive drive, double angleInDegrees) {
    this.drive = drive;
    this.targetAngle = Rotation2d.fromDegrees(angleInDegrees);
    addRequirements(drive);
  }

  @Override
  public void execute() {
    Rotation2d currentAngle = drive.getRotation();
    double angleDifference = targetAngle.minus(currentAngle).getRadians();
    
    // Determine direction to turn OwO
    double rotationDirection = Math.signum(angleDifference);
    
    // Slow down when close to target >w<
    double speed = Math.min(Math.abs(angleDifference), rotationSpeed) * rotationDirection;
    
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(drive.getRotation().minus(targetAngle).getRadians()) < angleTolerance;
  }
} 