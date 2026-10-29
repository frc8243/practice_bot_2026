// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final CommandXboxController driverXbox = new CommandXboxController(0);
    final CommandXboxController operatorXbox = new CommandXboxController(1);
    // The robot's subsystems and commands are defined here...
    //
    private SwerveSubsystem drivebase;
    private String serialNum = System.getenv("serialnum");
    private SwerveInputStream driveAngularVelocity;
    private SwerveInputStream driveDirectAngle;
    private SwerveInputStream driveRobotOriented;
    private SwerveInputStream driveAngularVelocityKeyboard;
    private SwerveInputStream driveDirectAngleKeyboard;
    private SendableChooser<Command> autoChooser;

    void createSwerveInputStreams() {
        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
         * velocity.
         */
        driveAngularVelocity =
                SwerveInputStream.of(
                                drivebase.getSwerveDrive(),
                                () -> driverXbox.getLeftY() * -1 * drivebase.getSpeedScale(),
                                () -> driverXbox.getLeftX() * -1 * drivebase.getSpeedScale())
                        .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative input
         * stream.
         */
        driveDirectAngle =
                driveAngularVelocity
                        .copy()
                        .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
                        .headingWhile(true);

        /**
         * Clone's the angular velocity input stream and converts it to a robotRelative input
         * stream.
         */
        driveRobotOriented =
                driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

        driveAngularVelocityKeyboard =
                SwerveInputStream.of(
                                drivebase.getSwerveDrive(),
                                () -> -driverXbox.getLeftY(),
                                () -> -driverXbox.getLeftX())
                        .withControllerRotationAxis(() -> driverXbox.getRawAxis(2) * -1)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);
        // Derive the heading axis with math!
        driveDirectAngleKeyboard =
                driveAngularVelocityKeyboard
                        .copy()
                        .withControllerHeadingAxis(
                                () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
                        .headingWhile(true)
                        .translationHeadingOffset(true)
                        .translationHeadingOffset(Rotation2d.fromDegrees(0));
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // the serial number to the alphabot is 031823E8
        // the serial number to the 2025 is 034159F4
        // 031823e8
        System.out.println("serialnum is " + serialNum);

        drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

        createSwerveInputStreams();
        // Configure the trigger bindings
        // shooter.setDefaultCommand(shooter.defaultBehaviorCommand());

        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        setupAuton();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        Command driveFieldOrientedAngularVelocity =
                drivebase.driveFieldOriented(driveAngularVelocity);

        Command driveFieldOrientedDirectAngleKeyboard =
                drivebase.driveFieldOriented(driveDirectAngleKeyboard);

        if (RobotBase.isSimulation()) {
            drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
        } else {
            drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
        }

        if (Robot.isSimulation()) {
            Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));

            driveDirectAngleKeyboard.driveToPose(
                    () -> target,
                    new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
                    new ProfiledPIDController(
                            5,
                            0,
                            0,
                            new Constraints(
                                    Units.degreesToRadians(360), Units.degreesToRadians(180))));
            driverXbox
                    .start()
                    .onTrue(
                            Commands.runOnce(
                                    () ->
                                            drivebase.resetOdometry(
                                                    new Pose2d(3, 3, new Rotation2d()))));
            driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
            driverXbox
                    .button(2)
                    .whileTrue(
                            Commands.runEnd(
                                    () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                    () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
        }

        driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));
        // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));

        // driverXbox
        // .b()
        // .whileTrue(
        // drivebase.snakeDriveCommand(
        // () -> -driverXbox.getLeftY(),
        // () -> -driverXbox.getLeftX(),
        // () -> {
        // double x = -driverXbox.getLeftX();
        // double y = -driverXbox.getLeftY();
        // if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1) {
        // return drivebase.getHeading().getRadians();
        // }
        // return Math.atan2(x, y);
        // }));

        driverXbox
                .rightBumper()
                .whileTrue(
                        drivebase.aimAtHub(
                                () -> -driverXbox.getLeftY(), () -> -driverXbox.getLeftX()));
    }

    // operatorXbox
    // .y()
    // .whileTrue(
    // arm.oscillateCommand(WristAngle.DEPLOY, WristAngle.SHAKE, 0.8) // 1.0
    // .alongWith(intake.in(-0.5)));

    // Press once to send the wrist to a position; it holds there until the operator
    // moves the left stick or presses the other position button.

    private static final Pose2d RIGHT_AUTO_START_POSE =
            new Pose2d(4.4, 0.7, Rotation2d.fromDegrees(0));
    private static final Pose2d MIDDLE_AUTO_START_POSE =
            new Pose2d(3.599, 4, Rotation2d.fromDegrees(0));
    private static final Pose2d OUTPOST_TRENCH_SHOOT_POSE =
            new Pose2d(3, 0.75, Rotation2d.fromDegrees(65));
    private static final Pose2d DEPOT_TRENCH_SHOOT_POSE =
            new Pose2d(3.21, 6.847, Rotation2d.fromDegrees(-65));
    private static final Pose2d DEPOT_STARTER = new Pose2d(4.4, 7.5, Rotation2d.fromDegrees(0));
    private static final Pose2d DEPOT_STRAIGHT = new Pose2d(8.2, 7.5, Rotation2d.fromDegrees(0));
    private static final Pose2d LEFT_AUTO_START_POSE =
            new Pose2d(4.4, 7.4, Rotation2d.fromDegrees(0));
    private static final Pose2d LEFT_AUTO_BACK_POSE =
            new Pose2d(4.4, 7.1, Rotation2d.fromDegrees(0));
    // private static final Pose2d LEFT_AUTO_START_POSE_SHOOT =
    // new Pose2d(4, 7.4, Rotation2d.fromDegrees(48));
    private static final Pose2d DEPOT_SIDE_POSE = new Pose2d(1.73, 6, Rotation2d.fromDegrees(180));
    private static final Pose2d DEPOT_SIDE_COLLECT_POSE =
            new Pose2d(1.0, 6, Rotation2d.fromDegrees(180));
    private static final Pose2d MIDDLE_SHOOT_POSE = new Pose2d(2.5, 4, Rotation2d.fromDegrees(0));
    private static final Pose2d OUTPOST_ZONE_POSE2D =
            new Pose2d(0.816, 0.716, Rotation2d.fromDegrees(180));
    private static final Pose2d OUTPOST_TRENCH_FOWARD =
            new Pose2d(7.6, 0.6, Rotation2d.fromDegrees(90));
    private static final Pose2d DEPOT_TRENCH_FOWARD =
            new Pose2d(7.6, 7.5, Rotation2d.fromDegrees(-90));
    private static final Pose2d OUTPOST_TRENCH_RUN =
            new Pose2d(7.6, 2.7, Rotation2d.fromDegrees(90));
    private static final Pose2d DEPOT_TRENCH_RUN =
            new Pose2d(7.6, 5.5, Rotation2d.fromDegrees(-90));
    private static final Pose2d NEUTRAL_POSE = new Pose2d(8.2, 4, Rotation2d.fromDegrees(-90));

    private void setupAuton() {
        autoChooser = new SendableChooser<>();

        Command Linedepot =
                Commands.sequence(
                        Commands.runOnce(
                                () -> drivebase.resetOdometryDeferredFlip(LEFT_AUTO_START_POSE)),
                        drivebase.driveToPoseDeferredWithFlip(DEPOT_TRENCH_FOWARD, 1),
                        drivebase.driveSlowlyToPoseDeferredWithFlip(DEPOT_TRENCH_RUN, 0.2),
                        drivebase.driveToPoseDeferredWithFlip(DEPOT_TRENCH_FOWARD, 0.8),
                        drivebase.driveToPoseDeferredWithFlip(LEFT_AUTO_START_POSE, 1),
                        drivebase.driveToPoseDeferredWithFlip(DEPOT_TRENCH_SHOOT_POSE, 0),
                        drivebase.driveToPoseDeferredWithFlip(LEFT_AUTO_START_POSE, 1));

        Command Line =
                Commands.sequence(
                        Commands.runOnce(() -> drivebase.resetOdometryDeferredFlip(DEPOT_STARTER)),
                        drivebase.driveToPoseDeferredWithFlip(DEPOT_STRAIGHT, 0));

        // autoChooser.addOption("drivestraight", driveStraight);

        autoChooser.addOption("Line", Line);

        autoChooser.setDefaultOption("donothing", Commands.none());
        SmartDashboard.putData("Autos/Selector", autoChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
