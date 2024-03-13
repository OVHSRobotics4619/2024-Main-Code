package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase {

  PhotonCamera cam;
  PhotonPoseEstimator photonPoseEstimator;

  public VisionSubsystem(PhotonCamera camera) {

    //Forward Camera
    this.cam = camera;

    // Construct PhotonPoseEstimator
    this.photonPoseEstimator = new PhotonPoseEstimator(Constants.AprilTags.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, Constants.AprilTags.CameraConstants.kRobotToCam);

  }

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = Constants.AprilTags.kSingleTagStdDevs;
    var targets = this.cam.getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
        var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
                tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = Constants.AprilTags.kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  public void updateGlobalPosition(SwerveSubsystem swerve) {
    System.out.println("Updating global position using apriltags...");
    Pose2d previousPose2d = swerve.getPose();
    System.out.println("Old Pose: " + previousPose2d.toString());
    Optional<EstimatedRobotPose> estimatedPosition = getEstimatedGlobalPose(previousPose2d);
    
    estimatedPosition.ifPresent(est -> {
      var estPose = est.estimatedPose.toPose2d();
      System.out.println("Estimated Pose: " + estPose.toString());
      // Change our trust in the measurement based on the tags we can see
      Matrix<N3, N1> estStdDevs = getEstimationStdDevs(estPose);

      swerve.addVisionReading(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
      System.out.println("Added vision reading to swerve!");
    });
  }

}