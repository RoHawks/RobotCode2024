package odomoteryLogging;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Class for odometry. Robot code should not use this directly- Instead, use the particular type for
 * your drivetrain (e.g., {@link DifferentialDriveOdometry}). Odometry allows you to track the
 * robot's position on the field over the course of a match using readings from encoders and a
 * gyroscope.
 *
 * <p>Teams can use odometry during the autonomous period for complex tasks like path following.
 * Furthermore, odometry can be used for latency compensation when using computer-vision systems.
 *
 * @param <T> Wheel positions type.
 */
public class OdometryLogging_Odometry<T extends WheelPositions<T>> {
  private final Kinematics<?, T> m_kinematics;
  private Pose2d m_poseMeters;

  private Rotation2d m_gyroOffset;
  private Rotation2d m_previousAngle;
  private T m_previousWheelPositions;

  /**
   * Constructs an Odometry object.
   *
   * @param kinematics The kinematics of the drivebase.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The current encoder readings.
   * @param initialPoseMeters The starting position of the robot on the field.
   */
  public OdometryLogging_Odometry(
      Kinematics<?, T> kinematics,
      Rotation2d gyroAngle,
      T wheelPositions,
      Pose2d initialPoseMeters) {
    m_kinematics = kinematics;
    m_poseMeters = initialPoseMeters;
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    m_previousAngle = m_poseMeters.getRotation();
    m_previousWheelPositions = wheelPositions.copy();
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The current encoder readings.
   * @param poseMeters The position on the field that your robot is at.
   */
  public void resetPosition(Rotation2d gyroAngle, T wheelPositions, Pose2d poseMeters) {
    m_poseMeters = poseMeters;
    m_previousAngle = m_poseMeters.getRotation();
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    m_previousWheelPositions = wheelPositions.copy();
  }

  /**
   * Returns the position of the robot on the field.
   *
   * @return The pose of the robot (x and y are in meters).
   */
  public Pose2d getPoseMeters() {
    return m_poseMeters;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and integration of the pose
   * over time. This method takes in an angle parameter which is used instead of the angular rate
   * that is calculated from forward kinematics, in addition to the current distance measurement at
   * each wheel.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The current encoder readings.
   * @return The new pose of the robot.
   */
  public Pose2d update(Rotation2d gyroAngle, T wheelPositions) {
    var angle = gyroAngle.plus(m_gyroOffset);
    SmartDashboard.putNumber("OD_Update_Angle_Degrees", angle.getDegrees());
    var twist = m_kinematics.toTwist2d(m_previousWheelPositions, wheelPositions);
    SmartDashboard.putNumber("OD_Update_Twist_dx", twist.dx);
    SmartDashboard.putNumber("OD_Update_Twist_dy", twist.dy);
    twist.dtheta = angle.minus(m_previousAngle).getRadians();
    SmartDashboard.putNumber("OD_Update_Twist_dtheta", twist.dtheta);

    var newPose = m_poseMeters.exp(twist);

    SmartDashboard.putNumber("OD_Update_NewPose_X", newPose.getX());
    SmartDashboard.putNumber("OD_Update_NewPose_Y", newPose.getY());
    SmartDashboard.putNumber("OD_Update_NewPose_Rotation", newPose.getRotation().getDegrees());


    m_previousWheelPositions = wheelPositions.copy();
    m_previousAngle = angle;
    m_poseMeters = new Pose2d(newPose.getTranslation(), angle);

    return m_poseMeters;
  }
}
