// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Datalogtester extends Command {
  private final Drive drive;
  private final Timer timer = new Timer();
  
  // Test configuration OwO
  private final double rampRate = 0.5;        // Volts per second for ramping
  private final double maxVoltage = 12.0;     // Maximum test voltage
  private final double rampUpTime = 4.0;      // Time to reach max voltage
  private final double steadyTime = 4.0;      // Time to hold max voltage
  private final double rampDownTime = 4.0;    // Time to return to 0
  private final double totalTime = rampUpTime + steadyTime + rampDownTime;

  // Data collection for feedforward calculation >w<
  private ArrayList<Double> voltages = new ArrayList<>();
  private ArrayList<Double> velocities = new ArrayList<>();
  private ArrayList<Double> timestamps = new ArrayList<>();
  private double kS = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;

  /** Creates a new Datalogtester. */
  public Datalogtester(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    drive.setPose(new Pose2d());
    voltages.clear();
    velocities.clear();
    timestamps.clear();
  }      

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = timer.get();
    double voltage = 0.0;

    // Calculate voltage based on phase UwU
    if (currentTime < rampUpTime) {
      voltage = (currentTime / rampUpTime) * maxVoltage;
    } else if (currentTime < rampUpTime + steadyTime) {
      voltage = maxVoltage;
    } else if (currentTime < totalTime) {
      double timeIntoRampDown = currentTime - (rampUpTime + steadyTime);
      voltage = maxVoltage * (1 - (timeIntoRampDown / rampDownTime));
    }

    // Apply voltage and collect data! >w<
    ChassisSpeeds speeds = new ChassisSpeeds(voltage, 0.0, 0.0);
    drive.runVelocity(speeds);

    // Store data points ʕ•ᴥ•ʔ
    voltages.add(voltage);
    velocities.add(drive.getRobotRelativeSpeeds().vxMetersPerSecond);
    timestamps.add(currentTime);

    // Calculate feedforward constants during the test
    calculateFeedforwardConstants();

    // Output current values to SmartDashboard
    SmartDashboard.putNumber("kS Estimate", kS);
    SmartDashboard.putNumber("kV Estimate", kV);
    SmartDashboard.putNumber("kA Estimate", kA);
  }

  private void calculateFeedforwardConstants() {
    if (voltages.size() < 2) return;

    // Calculate kS (look for the breakaway voltage) UwU
    for (int i = 0; i < velocities.size(); i++) {
      if (Math.abs(velocities.get(i)) > 0.01) {  // When robot starts moving
        kS = voltages.get(i);
        break;
      }
    }

    // Calculate kV (during steady state) (◕ᴗ◕✿)
    if (timer.get() > rampUpTime && timer.get() < (rampUpTime + steadyTime)) {
      double steadyStateVelocity = velocities.get(velocities.size() - 1);
      if (Math.abs(steadyStateVelocity) > 0.01) {
        kV = (maxVoltage - kS) / steadyStateVelocity;
      }
    }

    // Calculate kA (during acceleration) >w<
    if (timer.get() < rampUpTime && velocities.size() > 2) {
      int last = velocities.size() - 1;
      double deltaTime = timestamps.get(last) - timestamps.get(last - 1);
      double deltaVelocity = velocities.get(last) - velocities.get(last - 1);
      double acceleration = deltaVelocity / deltaTime;
      
      if (Math.abs(acceleration) > 0.01) {
        double velocity = velocities.get(last);
        double voltage = voltages.get(last);
        kA = (voltage - kS - kV * velocity) / acceleration;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    drive.stop();

    // Final feedforward values! ^w^
    System.out.println("Final Feedforward Constants:");
    System.out.println("kS: " + kS);
    System.out.println("kV: " + kV);
    System.out.println("kA: " + kA);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= totalTime;
  }
}
