// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FlapSubsystem;
import frc.robot.subsystems.PinSubsystem;
import com.pathplanner.lib.commands.PathPlannerAuto;

import java.io.File;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.shooter.*;
import frc.robot.commands.AutoFlap;
import frc.robot.commands.apriltags.PositionEstimation;
import frc.robot.commands.apriltags.TurnToTag2;
import frc.robot.commands.apriltags.TurnToTag3;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.climber.Extend;
import frc.robot.commands.climber.Hold;
import frc.robot.subsystems.VisionSubsystem;

//import frc.robot.commands.apriltags.TurnToTag;
//import frc.robot.commands.swervedrive.drivebase.AprilCommand;
//import frc.robot.commands.climber.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...

  private final PhotonCamera camera = new PhotonCamera(Constants.AprilTags.CameraConstants.kCameraName);

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  private final PinSubsystem pin = new PinSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem(camera, drivebase);
  private final FlapSubsystem flap = new FlapSubsystem();

  private final Shoot autoShoot = new Shoot(shooter);
  private final Intake intake = new Intake(shooter);
  private final Extend climbExtend = new Extend(climber, pin);
  private final Climb climb = new Climb(climber, pin);
  private final Hold hold = new Hold(climber, pin);
  private final PositionEstimation aprilPositionEstimation = new PositionEstimation(vision);



  // unused commands:
  /*
  private final Climb autoClimb = new Climb(climber);
  private final AprilCommand aprilCommand = new AprilCommand(camera, drivebase);
  private final TurnToTag pointToTag = new TurnToTag(camera, drivebase, 0);
  */

  private final Command demoPathCommand = drivebase.getAutonomousCommand("New Path", true);

  private final Command demoShootPathCommand = new PathPlannerAuto("Shoot demo");
  
  // point to tag V2 code starts here
  private TurnToTag3 pointToTag3 = new TurnToTag3(camera, drivebase);

  private AlignShoot alignShoot = new AlignShoot(drivebase, shooter, camera);
  private AlignShoot2 alignShoot2 = new AlignShoot2(drivebase, shooter, camera);

  private InstantCommand fullPoseReset = new InstantCommand(() -> drivebase.resetOdometry(new Pose2d()));
  private AutoFlap autoFlap = new AutoFlap(flap);
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(2);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);

  XboxController driverXbox = new XboxController(0);
  GenericHID buttonBoard = new GenericHID(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    NamedCommands.registerCommand("autoShoot", autoShoot);
    NamedCommands.registerCommand("alignShoot", alignShoot2);
    NamedCommands.registerCommand("autoFlap", autoFlap);
    NamedCommands.registerCommand("fullPoseReset", fullPoseReset);
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                                                    // Applies deadbands and inverts controls because joysticks
                                                    // are back-right positive while robot
                                                    // controls are front-left positive
                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                  Constants.OperatorConstants.LEFT_Y_DEADBAND),
                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                  Constants.OperatorConstants.LEFT_X_DEADBAND),
                                                    () -> -driverXbox.getRightX(),
                                                    () -> -driverXbox.getRightY());

    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
                                                    () ->
                                                        MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                              Constants.OperatorConstants.LEFT_Y_DEADBAND),
                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                Constants.OperatorConstants.LEFT_X_DEADBAND),
                                                    () -> driverXbox.getRawAxis(2));

    @SuppressWarnings("unused")
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                              Constants.OperatorConstants.LEFT_Y_DEADBAND),
                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                Constants.OperatorConstants.LEFT_X_DEADBAND),
                                                    () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                Constants.OperatorConstants.RIGHT_X_DEADBAND), 
                                                    driverXbox::getYButtonPressed, 
                                                    driverXbox::getAButtonPressed, 
                                                    driverXbox::getXButtonPressed, 
                                                    driverXbox::getBButtonPressed);
    
    @SuppressWarnings("unused")
    TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                 Constants.OperatorConstants.LEFT_Y_DEADBAND),
                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                 Constants.OperatorConstants.LEFT_X_DEADBAND),
                                                    () -> driverXbox.getRawAxis(2), () -> true);

    @SuppressWarnings("unused")
    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(driverController.getRawAxis(1), Constants.OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getRawAxis(0), Constants.OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverController.getRawAxis(2), () -> true);

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDrive : closedFieldAbsoluteDrive);
    //drivebase.setDefaultCommand(closedAbsoluteDriveAdv);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Trigger rightTrigger = new Trigger(() -> driverXbox.getRawAxis(Constants.OIConstants.RIGHT_TRIGGER) > 0.05);
    Trigger leftTrigger = new Trigger(() -> driverXbox.getRawAxis(Constants.OIConstants.LEFT_TRIGGER) > 0.05);

    Trigger funStickUp = new Trigger(() -> buttonBoard.getRawAxis(1) < 0);
    Trigger funStickDown = new Trigger(() -> buttonBoard.getRawAxis(1) > 0);

    Trigger buttonBoardL2 = new Trigger(() -> buttonBoard.getRawAxis(2) > 0);
    Trigger buttonBoardR2 = new Trigger(() -> buttonBoard.getRawAxis(3) > 0);



    // rightTrigger.onTrue(autoShoot);                                         // done
    
    new JoystickButton(buttonBoard, Constants.buttonBoard.A)
                      .whileTrue(autoShoot);

    rightTrigger
                      .whileTrue(autoShoot);

    new JoystickButton(buttonBoard, Constants.buttonBoard.B)             // done
                      .whileTrue(alignShoot2);

    new JoystickButton(driverXbox, Constants.OIConstants.BACK)              // done
                      .onTrue((new InstantCommand(drivebase::zeroGyro)));

    buttonBoardR2                                                             // done
                      .whileTrue(intake);

    leftTrigger                                                             // done
                      .whileTrue(intake);

    new JoystickButton(buttonBoard, Constants.buttonBoard.R1)          // done
                      .onTrue(new InstantCommand(shooter::amp))
                      .onFalse(new InstantCommand(shooter::stopAll));

    /*new JoystickButton(driverXbox, Constants.OIConstants.A)
                      .whileTrue(climb)
                      .whileFalse(hold);

    new JoystickButton(driverXbox, Constants.OIConstants.Y)
                  .whileTrue(climbExtend);
    */

    funStickDown
                      .whileTrue(climb)
                      .whileFalse(hold);

    funStickUp
                      .onTrue(new InstantCommand(() -> {
                        hold.cancel();
                        System.out.println("Hold cancelled by stick");}))
                      .whileTrue(climbExtend);
    /*
    new JoystickButton(driverXbox, Constants.OIConstants.R_BUMPER)          // test
                      .whileTrue(demoPathCommand);

    new JoystickButton(driverXbox, Constants.OIConstants.X)
                      .whileTrue(aprilPositionEstimation);                 // test
     */

    buttonBoardL2
                      .onTrue(new InstantCommand(flap::enableFlap))
                      .onFalse(new InstantCommand(flap::disableFlap));

    new JoystickButton(buttonBoard, Constants.buttonBoard.L1)
                      .onTrue(new InstantCommand(flap::reverseFlap))
                      .onFalse(new InstantCommand(flap::disableFlap));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    String autoName = "Full shoot auto blue";

    PathPlannerAuto thisAuto = new PathPlannerAuto(autoName);

    drivebase.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(autoName)); 
    return thisAuto;
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}