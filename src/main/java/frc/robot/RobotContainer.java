// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BackShootLevel3;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShootLevel2;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Wrist.Wrist;

import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

        final CommandXboxController driverController = new CommandXboxController(0);
        private final Wrist wrist = new Wrist();
        private final Swerve drivebase = new Swerve(new File(Filesystem.getDeployDirectory(),
                        "swerve"));
        private final Climb climb = new Climb();
        private final Arm arm = new Arm();


        private final BackShootLevel3 backShootLevel3 = new BackShootLevel3(arm, wrist);
        private final ShootLevel2 shootLevel2 = new ShootLevel2(arm, wrist);
        private final IntakeCmd IntakeCmd = new IntakeCmd(arm, wrist);
        
        



        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverController.getLeftY() * -1,
                        () -> driverController.getLeftX() * -1)
                        .withControllerRotationAxis(driverController::getRightX)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .scaleRotation(0.7)
                        .allianceRelativeControl(true);

        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverController::getRightX,
                                        driverController::getRightY)
                        .headingWhile(true);

        Command driveFieldOrientedDriectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
        Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

        // Non reality code
        SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX())
                        .withControllerRotationAxis(() -> driverController.getRawAxis(
                                        2))
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);
        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        driverController.getRawAxis(
                                                        2) * Math.PI)
                                        * (Math.PI * 2),
                                        () -> Math.cos(
                                                        driverController.getRawAxis(
                                                                        2) * Math.PI)
                                                        *
                                                        (Math.PI * 2))
                        .headingWhile(true);

        Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

        Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);
                NamedCommands.registerCommand("ShootLevel2", shootLevel2);

        }

        private void configureBindings() {
                // drivebase.setDefaultCommand(
                // Robot.isSimulation() ? driveFieldOrientedAnglularVelocityKeyboard :
                // driveFieldOrientedAnglularVelocity);
                drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
                driverController.button(5).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
                driverController.button(10).onTrue((Commands.runOnce(drivebase::zeroGyro)));
                driverController.button(4).whileTrue(drivebase.centerModulesCommand());
                driverController.button(6).toggleOnTrue(shootLevel2);
                driverController.button(7).toggleOnTrue(backShootLevel3);
                driverController.button(1).toggleOnTrue(IntakeCmd);
                // driverController.button(2).whileTrue(
                /// drivebase.driveToPose(new Pose2d(new Translation2d(7, 4),
                // Rotation2d.fromDegrees(0))));
                driverController.button(8).whileTrue(new InstantCommand(() -> climb.openClimb()))
                                .whileFalse(new InstantCommand(() -> climb.stopOpener()));
                driverController.button(9).whileTrue(new InstantCommand(() -> climb.closeClimb()))
                                .whileFalse(new InstantCommand(() -> climb.stopCloser()));
                // driverController.button(1).whileTrue(drivebase.sysIdAngleMotorCommand());
                // driverController.button(3).onTrue(drivebase.driveToDistanceCommand(1, 3));
        }

        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return drivebase.getAutonomousCommand("New Auto");
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}