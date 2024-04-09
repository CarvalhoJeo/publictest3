package frc.Java_Is_UnderControl.Swerve;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.Java_Is_UnderControl.Control.PidConfig;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimator;

public class OdometryEnabledSwerveConfig extends BaseSwerveConfig {

  public final PoseEstimator autonomousPoseEstimator;

  public final PoseEstimator teleoperatedPoseEstimator;

  public final PidConfig moveToPosePidConfig;

  public OdometryEnabledSwerveConfig(double maxTranslationSpeed, double maxRotationSpeed, Alliance defaultAlliance,
      SwervePathPlannerConfig pathPlannerConfig, PoseEstimator autonomousPoseEstimator, PoseEstimator teleoperatedPoseEstimator, PidConfig moveToPosePidConfig) {
    super(maxTranslationSpeed, maxRotationSpeed, defaultAlliance, pathPlannerConfig);
    this.autonomousPoseEstimator = autonomousPoseEstimator;
    this.teleoperatedPoseEstimator = teleoperatedPoseEstimator;
    this.moveToPosePidConfig = moveToPosePidConfig;
  }
  
}
