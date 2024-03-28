package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.apriltags.TurnToTag3;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import org.photonvision.PhotonCamera;

public class AlignShoot extends SequentialCommandGroup  {
  public AlignShoot(SwerveSubsystem swerve, ShooterSubsystem shooter, PhotonCamera camera) {
    addCommands(
      new TurnToTag3(camera, swerve),
      new Shoot(shooter)
    );
  }
};