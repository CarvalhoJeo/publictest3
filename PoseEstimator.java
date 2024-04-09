package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveSubsystem;

public interface PoseEstimator {

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose);

  public void setSwerveDriveForGyroReference(OdometryEnabledSwerveSubsystem swerve);
}
