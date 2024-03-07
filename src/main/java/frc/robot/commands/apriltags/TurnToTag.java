package frc.robot.commands.apriltags;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class TurnToTag extends PIDCommand {

    public TurnToTag(PhotonCamera camera, SwerveSubsystem swerve, double targetAngleDegrees) {
        super(
          new PIDController(3, 1, 0),
          // Close loop on heading
          () -> {
            PhotonPipelineResult res = camera.getLatestResult();
            if (res.hasTargets()) {
              return res.getBestTarget().getYaw();
            } else {
              System.out.println("Apriltag turning canceled (lost vision)");
              return 0.0;
            }
          },
          // Set reference to target
          targetAngleDegrees,
          // Pipe output to turn robot
          output -> {
            swerve.drive(new Translation2d(), Math.toRadians(output), false);
            System.out.println("Turn velocity: " + output);
          },
          // Require the swervedrive
          swerve);

          // Set the controller to be continuous (because it is an angle controller)
          getController().enableContinuousInput(-180, 180);
          // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
          // setpoint before it is considered as having reached the reference
          getController()
              .setTolerance(1, 7);
    }

    @Override
    public void initialize() {
      System.out.println("Init turn command");
    }
    
    @Override
    public void end(boolean interrupted) {
      System.out.println("Ending turn command");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return getController().atSetpoint();
    }
};