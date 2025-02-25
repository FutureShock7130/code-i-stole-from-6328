// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Datalogtester;
import frc.robot.commands.move1m;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.Vision;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
// import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision = new Vision();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new Swerve(0),
                new Swerve(1),
                new Swerve(2),
                new Swerve(3),
                vision);

        break;

      // case SIM:
      //   // Sim robot, instantiate physics sim IO implementations
      //   drive =
      //       new Drive(
      //           new GyroIO() {},
      //           new ModuleIOSim(),
      //           new ModuleIOSim(),
      //           new ModuleIOSim(),
      //           new ModuleIOSim());
      //   break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                vision);
        break;
    }

    // Set up auto routines

    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.setDefaultOption("leave", new PathPlannerAuto("LEAVE"));
    autoChooser.addOption("full auto", new PathPlannerAuto("FullAuto"));
    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData("auto", autoChooser);

    // Add move1m command button to SmartDashboard! >w<
    SmartDashboard.putData("Move 1 meter forward!", new move1m(drive));
    
    // Add Datalogtester button to SmartDashboard! ʕ•ᴥ•ʔ
    SmartDashboard.putData("Run Feedforward Test!", new Datalogtester(drive));
    
    // Add turn-to-angle buttons! ʕ•ᴥ•ʔ
    SmartDashboard.putData("Turn 0°", new TurnToAngle(drive, 0));
    SmartDashboard.putData("Turn 60°", new TurnToAngle(drive, 60));
    SmartDashboard.putData("Turn 120°", new TurnToAngle(drive, 120));
    SmartDashboard.putData("Turn 180°", new TurnToAngle(drive, 180));
    SmartDashboard.putData("Turn 240°", new TurnToAngle(drive, 240));
    SmartDashboard.putData("Turn 300°", new TurnToAngle(drive, 300));

    // Add our kawaii reset button! ✧˖°
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    driveTab.add("Reset Pose/Gyro", Commands.runOnce(() -> drive.resetPoseToZero()))
        .withSize(2, 1)
        .withPosition(0, 0);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    drive.setPose(new Pose2d());
    return autoChooser.getSelected();
  }
}
