package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase {

  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonCamera cam;
  Transform3d robotToCam;
  PhotonPoseEstimator photonPoseEstimator;

  public VisionSubsystem(PhotonCamera camera) {

    // The field from AprilTagFields will be different depending on the game.
    this.aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    //Forward Camera
    this.cam = camera;
    this.robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    
    // Construct PhotonPoseEstimator
    this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, robotToCam);

  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  public void updateGlobalPosition(SwerveSubsystem swerve) {
    Pose2d previousPose2d = swerve.getPose();
    Optional<EstimatedRobotPose> estimatedPosition = getEstimatedGlobalPose(previousPose2d);
    if(estimatedPosition != null) {
      //swerve.addVisionReading(estimatedPosition.estimatedPose);
    }
  }

}