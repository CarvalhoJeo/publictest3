package frc.Java_Is_UnderControl.Swerve;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BaseSwerveConfig {

  public final double maxTranslationSpeed;

  public final double maxRotationSpeed;

  public final Alliance defaultAlliance;

  public final SwervePathPlannerConfig pathPlannerConfig;

  public BaseSwerveConfig(double maxTranslationSpeed, double maxRotationSpeed, Alliance defaultAlliance,
      SwervePathPlannerConfig pathPlannerConfig) {
    this.maxTranslationSpeed = maxTranslationSpeed;
    this.maxRotationSpeed = maxRotationSpeed;
    this.defaultAlliance = defaultAlliance;
    this.pathPlannerConfig = pathPlannerConfig;
  }
  
}
