// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.BackShootLevel3;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShootLevel2;
import frc.robot.commands.WristReset;
import frc.robot.subsystems.RobotStatusManager;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Drive.AprilTagAligner;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Wrist.Wrist;

import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {
        private final UsbCamera usbcam = CameraServer.startAutomaticCapture();

        final CommandPS5Controller driverController = new CommandPS5Controller(0);
        final CommandXboxController operatorController = new CommandXboxController(1);
        private final Wrist wrist = new Wrist();
        private final Swerve drivebase = new Swerve(new File(Filesystem.getDeployDirectory(),
                        "swerve"));
        private final Climb climb = new Climb();
        private final Arm arm = new Arm();
        private final AprilTagAligner align = new AprilTagAligner("back", drivebase);
        private final RobotStatusManager robotStatusManager = new RobotStatusManager();

        private final AlignCommand alignCommand = new AlignCommand(drivebase, align);
        private final BackShootLevel3 backShootLevel3 = new BackShootLevel3(arm, wrist, robotStatusManager);
        private final ShootLevel2 shootLevel2 = new ShootLevel2(arm, wrist, robotStatusManager);
        private final IntakeCmd IntakeCmd = new IntakeCmd(arm, wrist, robotStatusManager);
        private final WristReset WristReset = new WristReset(wrist,arm);

        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverController.getLeftY() * -1,
                        () -> driverController.getLeftX() * -1)
                        .withControllerRotationAxis(() -> driverController.getRightX() * -1)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .scaleRotation(0.7)
                        .allianceRelativeControl(true);

        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverController::getRightX,
                                        driverController::getRightY)
                        .headingWhile(true);

        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
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
        private final SendableChooser<Command> m_Chooser = new SendableChooser<>();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                usbcam.setResolution(600, 400);
                // Configure the trigger bindings
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);
                NamedCommands.registerCommand("ShootLevel2", shootLevel2);
                NamedCommands.registerCommand("L1", new InstantCommand(() ->arm.setSetPoint(-15)));
                NamedCommands.registerCommand("SetDefault", new InstantCommand(() ->arm.setSetPoint(-82.65)));
                m_Chooser.setDefaultOption("Taxi", drivebase.getAutonomousCommand("Taxi"));
                m_Chooser.addOption("Right-L2", drivebase.getAutonomousCommand("Right-L2"));
                m_Chooser.addOption("Turn90", drivebase.getAutonomousCommand("Turn90"));
                m_Chooser.addOption("middle-l1-choreo", drivebase.getAutonomousCommand("middle-l1-choreo"));
                m_Chooser.addOption("middle-l1", drivebase.getAutonomousCommand("middle-l1"));
                m_Chooser.addOption("Right-L2-Choreo", drivebase.getAutonomousCommand("Right-L2-Choreo"));
                

                SmartDashboard.putData("Auto Selector", m_Chooser);

        }

        private void configureBindings() {
                drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
                driverController.pov(90).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
                driverController.pov(270).onTrue(alignCommand);
                driverController.pov(0).whileTrue(drivebase.centerModulesCommand());
                driverController.button(7).onTrue(IntakeCmd);
                driverController.button(6).onTrue(shootLevel2);
                driverController.button(8).onTrue(backShootLevel3);
                driverController.button(1).whileTrue(new InstantCommand(() -> wrist.setWheelMotor(9)))
                                .whileFalse(new InstantCommand(() -> wrist.setWheelMotor(0)));
                driverController.button(3).whileTrue(new InstantCommand(() -> wrist.setWheelMotor(-11)))
                                .whileFalse(new InstantCommand(() -> wrist.setWheelMotor(0)));
                driverController.button(2).onTrue(new InstantCommand(
                                () -> arm.setSetPoint(Constants.LevelAngles.DefaultAngle))
                                .alongWith(new InstantCommand(
                                                () -> wrist.setSetPoint(Constants.LevelAngles.DefaultAngleWrist))));
                driverController.button(5).whileTrue(driveRobotOrientedAngularVelocity);
                operatorController.pov(270).onTrue(WristReset);
                operatorController.pov(0).onTrue(new InstantCommand(() -> arm.setSetPoint(Constants.LevelAngles.DefaultAngle)));
                operatorController.pov(90).onTrue(new InstantCommand(() -> backShootLevel3.cancel()));
                operatorController.pov(180).onTrue(new InstantCommand(() -> wrist.resetAngle()));
                operatorController.button(8).onTrue(new InstantCommand(() -> drivebase.zeroGyro()));
                operatorController.button(3).onTrue(new InstantCommand(() -> IntakeCmd.cancel()));
                operatorController.button(4).whileTrue(new InstantCommand(() -> climb.openClimb()))
                                .whileFalse(new InstantCommand(() -> climb.stopOpener()));
                operatorController.button(1).whileTrue(new InstantCommand(() -> climb.closeClimb()))
                                .whileFalse(new InstantCommand(() -> climb.stopCloser()));
                operatorController.button(2)
                                .onTrue(new InstantCommand(() -> arm.setSetPoint(Constants.LevelAngles.DefaultAngle)));
                operatorController.button(10).onTrue(new InstantCommand(
                                () -> arm.setSetPoint(Constants.LevelAngles.BackLevel3))
                                .alongWith(new InstantCommand(
                                                () -> wrist.setSetPoint(Constants.LevelAngles.BackLevel3Wrist))));
                operatorController.button(6).onTrue(new InstantCommand(
                                () -> arm.setSetPoint(Constants.LevelAngles.Level2))
                                .alongWith(new InstantCommand(
                                                () -> wrist.setSetPoint(Constants.LevelAngles.DefaultAngle))));
                operatorController.button(7).onTrue(new InstantCommand(() -> arm.setSetPoint(10)));
                operatorController.button(9).whileTrue(new InstantCommand(() -> climb.setCloser(-1)))
                                .whileFalse(new InstantCommand(() -> climb.setCloser(0)));

        }

        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return m_Chooser.getSelected();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
        
        public void centerModules() {
                drivebase.centerModulesCommand();
        }
}