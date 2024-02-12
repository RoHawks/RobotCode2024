// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import universalSwerve.SwerveDrive;
import universalSwerve.SwerveFactory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.SparkMaxPIDController;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class RobotPrototype extends TimedRobot {
  private SparkMaxAbsoluteEncoder mLowerJointAbsoluteEncoder;
  private SparkMaxAbsoluteEncoder mUpperJointAbsoluteEncoder;
  private RelativeEncoder mLowerJointRelativeEncoder;
  private XboxController mController;
  private CANSparkMax mLowerJointSpark;
  private CANSparkMax mUpperJointSpark;
  private SparkMaxPIDController mLowerJointPIDController;
  private SparkMaxPIDController mUpperJointPIDController;

  private CANSparkMax mUpperIntakeSpark;
  private CANSparkMax mLowerIntakeSpark;
  

  private TrapezoidProfile.Constraints mLowerJointConstraints;
  private TrapezoidProfile.State mLowerJointGoal;
  private TrapezoidProfile.State mLowerJointSetpoint;

  private TrapezoidProfile.Constraints mUpperJointConstraints;
  private TrapezoidProfile.State mUpperJointGoal;
  private TrapezoidProfile.State mUpperJointSetpoint;

  private CANSparkMax mNETurnSparkMax;
  private CANSparkMax mSETurnSparkMax;
  private CANSparkMax mSWTurnSparkMax;
  private CANSparkMax mNWTurnSparkMax;
  private CANSparkMax[] mTurnSparkMaxes;
  
  private TalonFX mNEDriveFalcon;
  private TalonFX mSEDriveFalcon;
  private TalonFX mSWDriveFalcon;
  private TalonFX mNWDriveFalcon;

  private TalonFX[] mDriveFalcons;

  private Counter mLampreyNE;
  private Counter mLampreySE;
  private Counter mLampreySW;
  private Counter mLampreyNW;

  private Counter[] mLampreys;
 
  private SwerveDrive mSwerveDrive;
  

  public void fullRobotInit()
  {
    mController = new XboxController(2);
    robotInitForSwerve();
    robotInitForIntakeAndArm();
  }

  public void robotInit()
  {
    //robotInitForSwerve();
    //robotInitForIntakeAndTestsAndArm();
    fullRobotInit();
  }

  public void robotInitForSwerve()
  {
    mController = new XboxController(2);
    mSwerveDrive = SwerveFactory.Create2022Swerve();
    mSwerveDrive.Initialize();
    
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  public void robotInitForIntakeAndArm() {
    
    mLowerJointSpark = new CANSparkMax(3, MotorType.kBrushless);
    mLowerJointSpark.restoreFactoryDefaults();
    mLowerJointSpark.setSmartCurrentLimit(40);
    mLowerJointSpark.setIdleMode(IdleMode.kBrake);
    
    mUpperJointSpark = new CANSparkMax(4, MotorType.kBrushless);
    mUpperJointSpark.restoreFactoryDefaults();
    mUpperJointSpark.setSmartCurrentLimit(40);
    mUpperJointSpark.setIdleMode(IdleMode.kBrake);

    mLowerJointAbsoluteEncoder = mLowerJointSpark.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    mLowerJointAbsoluteEncoder.setZeroOffset(Constants.LOWER_JOINT_ENCODER_ZERO);

    mUpperJointAbsoluteEncoder = mUpperJointSpark.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    mUpperJointAbsoluteEncoder.setZeroOffset(0.75);

    mLowerJointSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    mLowerJointSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    
    mUpperJointSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    mUpperJointSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    mLowerJointSpark.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)LOWER_JOINT_FORWARDS_LIMIT);  
    mLowerJointSpark.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)LOWER_JOINT_REVERSE_LIMIT);
    
    mUpperJointSpark.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)UPPER_JOINT_FORWARD_LIMIT);  
    mUpperJointSpark.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)UPPER_JOINT_REVERSE_LIMIT);

    mLowerJointPIDController = mLowerJointSpark.getPIDController();
    mLowerJointPIDController.setFeedbackDevice(mLowerJointAbsoluteEncoder);
    
    mUpperJointPIDController = mUpperJointSpark.getPIDController();
    mUpperJointPIDController.setFeedbackDevice(mUpperJointAbsoluteEncoder);

    mLowerJointPIDController.setP(4.5);
    mLowerJointPIDController.setI(0);
    mLowerJointPIDController.setD(0);
    mLowerJointPIDController.setFF(0);

    mUpperJointPIDController.setP(6.0);
    mUpperJointPIDController.setI(0);
    mUpperJointPIDController.setD(0);
    mUpperJointPIDController.setFF(0);

    
    mLowerJointConstraints = new TrapezoidProfile.Constraints(1, 0.25);
    mLowerJointSetpoint = new TrapezoidProfile.State(mLowerJointAbsoluteEncoder.getPosition(), 0);
    
    mUpperJointConstraints = new TrapezoidProfile.Constraints(2, 0.5);
    mUpperJointSetpoint = new TrapezoidProfile.State(mUpperJointAbsoluteEncoder.getPosition(), 0);

    mUpperIntakeSpark = new CANSparkMax(13, MotorType.kBrushless);
    mUpperIntakeSpark.restoreFactoryDefaults();
    mUpperIntakeSpark.setSmartCurrentLimit(20);

    
    mLowerIntakeSpark = new CANSparkMax(16, MotorType.kBrushless);
    mLowerIntakeSpark.restoreFactoryDefaults();
    mLowerIntakeSpark.setSmartCurrentLimit(20);

  }

  private Counter InitializeLamprey(int pPort)
  {
      Counter returnValue = new Counter(new DigitalInput(pPort));
      returnValue.setSemiPeriodMode(true);
      returnValue.setSamplesToAverage(1);
      return returnValue;

  }

  private TalonFX InitializeDriveFalcon(int pCANID)
  {
    TalonFX returnValue = new TalonFX(pCANID);
    return returnValue;

  }


    private CANSparkMax InitializeSparkMaxForSwerveTurn(int pCANID)
    {
      CANSparkMax returnValue = new CANSparkMax(pCANID, MotorType.kBrushless);
      returnValue.setSmartCurrentLimit(20);
      returnValue.setIdleMode(IdleMode.kBrake);
      //on real robot, burn flash and limit CAN frame rate
      return returnValue;
    }

    private int mSwerveTurnBringUpCounter = 0;
    private boolean mLastStartButtonPushed = false;
    private void SwerveTurnBringUp()
    {
      if(!mLastStartButtonPushed && mController.getStartButton())
      {
        mSwerveTurnBringUpCounter++;
        mSwerveTurnBringUpCounter = mSwerveTurnBringUpCounter % mTurnSparkMaxes.length;
      }
      mLastStartButtonPushed =  mController.getStartButton();

      for(int i = 0; i < mTurnSparkMaxes.length; i++)
      {
        if(i == mSwerveTurnBringUpCounter)
        {
          mTurnSparkMaxes[i].set(mController.getRightTriggerAxis() - mController.getLeftTriggerAxis());
        }
        else
        {
          mTurnSparkMaxes[i].set(0);
        }
      }

      for(int i = 0; i < mLampreys.length; i++)
      {
        SmartDashboard.putNumber("DIO_Period_" + Integer.toString(i), mLampreys[i].getPeriod());
      }
    }

    private int mSwerveDriveBringUpCounter = 0;    
    private void SwerveDriveBringUp()
    {
      if(!mLastStartButtonPushed && mController.getStartButton())
      {
        mSwerveDriveBringUpCounter++;
        mSwerveDriveBringUpCounter = mSwerveDriveBringUpCounter % mDriveFalcons.length;
      }
      mLastStartButtonPushed =  mController.getStartButton();

      for(int i = 0; i < mDriveFalcons.length; i++)
      {
        if(i == mSwerveDriveBringUpCounter)
        {
          mDriveFalcons[i].set(ControlMode.PercentOutput, mController.getRightTriggerAxis() - mController.getLeftTriggerAxis());
        }
        else
        {
          mDriveFalcons[i].set(ControlMode.PercentOutput, 0);
        }
      }
    }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

      try
      {
      SmartDashboard.putNumber("UpperJoint_AbsoluteEncoder", mUpperJointAbsoluteEncoder.getPosition());
      SmartDashboard.putNumber("UpperJoint_AppliedOutput", mUpperJointSpark.getAppliedOutput());
      SmartDashboard.putNumber("UpperJoint_Current", mUpperJointSpark.getOutputCurrent());

      SmartDashboard.putNumber("LowerJoint_AbsoluteEncoder", mLowerJointAbsoluteEncoder.getPosition());
      SmartDashboard.putNumber("LowerJoint_AppliedOutput", mLowerJointSpark.getAppliedOutput());
      SmartDashboard.putNumber("LowerJoint_Current", mLowerJointSpark.getOutputCurrent());
      }
      catch(Exception e)
      {

      }

    

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
 
  } 
    
    // In rotations
  private final static double LOWER_JOINT_FORWARDS_LIMIT = 0.50;
  private final static double LOWER_JOINT_INTAKE_LOCATION = 0.48;
  private final static double LOWER_JOINT_SCORE_LOCATION = 0.28;
  private final static double LOWER_JOINT_REVERSE_LIMIT = 0.26;
  


  private final static double UPPER_JOINT_FORWARD_LIMIT = 0.70;
  private final static double UPPER_JOINT_INTAKE_LOCATION = 0.66;
  private final static double UPPER_JOINT_SCORE_LOCATION = 0.13;
  private final static double UPPER_JOINT_REVERSE_LIMIT = 0.11;

  public void teleopPeriodicRunIntake()
  {
    mLowerIntakeSpark.set(-0.5);
    mUpperIntakeSpark.set(-1.0);
  }


  public void teleopPeriodic_BothArmsSmartMotion()
  {
    if(mController.getAButton() || mController.getBButton())
    {
      teleopPeriodic_LowerJointMotionProfile();
      teleopPeriodic_UpperJointMotionProfile();
    }
    else
    {
      mLowerJointSpark.stopMotor();
      mUpperJointSpark.stopMotor();
    }
  }

  private double mLowerJointIdealEndSpot = LOWER_JOINT_INTAKE_LOCATION;
  public void teleopPeriodic_LowerJointMotionProfile()
  {
    double kDt = 0.02;
    
    if(mController.getAButton())
    {
      mLowerJointIdealEndSpot = LOWER_JOINT_SCORE_LOCATION;
    }    
    else if(mController.getBButton())
    {
      mLowerJointIdealEndSpot = LOWER_JOINT_INTAKE_LOCATION;
    }

    mLowerJointGoal = new TrapezoidProfile.State(mLowerJointIdealEndSpot, 0);

     // Create a motion profile with the given maximum velocity and maximum
    // acceleration constraints for the next setpoint, the desired goal, and the
    // current setpoint.
    var profile = new TrapezoidProfile(mLowerJointConstraints, mLowerJointGoal, mLowerJointSetpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    mLowerJointSetpoint = profile.calculate(kDt);

    // Send setpoint to offboard controller PID
    mLowerJointPIDController.setReference(mLowerJointSetpoint.position,ControlType.kPosition);

  }

  public void RunSwerve()
  {
    mSwerveDrive.StandardSwerveDrive(mController.getLeftX(), mController.getLeftY(), mController.getRightTriggerAxis(), mController.getRightX(), false, 180);
  }

  public void teleopPeriodic()
  {
    RunSwerve();
    //SwerveTurnBringUp();
    //SwerveDriveBringUp();

    teleopPeriodicRunIntake();
  }

  double mIdealEndSpotUpperJoint = UPPER_JOINT_INTAKE_LOCATION;
  public void teleopPeriodic_UpperJointMotionProfile()
  {
    double kDt = 0.02;

    if(mController.getAButton())
    {
      mIdealEndSpotUpperJoint = UPPER_JOINT_SCORE_LOCATION;
    }    
    else if(mController.getBButton())
    {
      mIdealEndSpotUpperJoint = UPPER_JOINT_INTAKE_LOCATION;
    }

    mUpperJointGoal = new TrapezoidProfile.State(mIdealEndSpotUpperJoint, 0);

    // Create a motion profile with the given maximum velocity and maximum
    // acceleration constraints for the next setpoint, the desired goal, and the
    // current setpoint.
    var profile = new TrapezoidProfile(mUpperJointConstraints, mUpperJointGoal, mUpperJointSetpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    mUpperJointSetpoint = profile.calculate(kDt);

    // Send setpoint to offboard controller PID
    mUpperJointPIDController.setReference(mUpperJointSetpoint.position,ControlType.kPosition);
  }

  
  public void teleopPeriodic_LowerArmBasicPID()
  {

        double val = 0.32;
        if(mController.getAButton())
        {
          val = 0.32;
        }     
        else if(mController.getBButton())
        {
          val = 0.48;
        }
        mLowerJointPIDController.setReference(val, ControlType.kPosition);

        SmartDashboard.putNumber("AbsoluteEncoder", mLowerJointAbsoluteEncoder.getPosition());
        SmartDashboard.putNumber("AppliedOutput", mLowerJointSpark.getAppliedOutput());

  }


  public void teleopPeriodic_UpperJointPID()
  {    
    if(mController.getAButton())
    {
      mUpperJointPIDController.setReference(UPPER_JOINT_SCORE_LOCATION, ControlType.kPosition);    
    }
    else if(mController.getBButton())
    {
      mUpperJointPIDController.setReference(UPPER_JOINT_INTAKE_LOCATION, ControlType.kPosition); 
    }
    
    
  }



  public void teleopPeriodic_ManualControlUpperJoint()
  {    
    double speed = mController.getRightTriggerAxis() - mController.getLeftTriggerAxis();
    SmartDashboard.putNumber("Calling set", speed);
    mUpperJointSpark.set(speed);

  }




  /** This function is called periodically during operator control. */
  
  public void teleopPeriodicManualLowerArmCotnrol() {

    
    double speed = mController.getRightTriggerAxis() - mController.getLeftTriggerAxis();
    mLowerJointSpark.set(speed);
    

    /*
    double speed = mController.getRightTriggerAxis() - mController.getLeftTriggerAxis();
    SmartDashboard.putNumber("Speed: ", speed);
    double relativeEncoderPosition = mLowerJointSpark.getEncoder().getPosition();
    double absoluteEncoderPosition = mLowerJointAbsoluteEncoder.getPosition();
    
    SmartDashboard.putNumber("Relative Encoder Position", relativeEncoderPosition);
    SmartDashboard.putNumber("Absolute Encoder Position", absoluteEncoderPosition);

    mLowerJointSpark.set(speed);
    */
/*
      if(mController.getXButton() && !mXLastPressed)
      {
          val += 0.1;
      }
      mXLastPressed = mController.getXButton();

      if(mController.getYButton() && !mYLastPressed)
      {
          val -= 0.1;
      }
      mYLastPressed = mController.getYButton();
      mLowerJointPIDController.setReference(val, ControlType.kPosition);
  */
  /*
      double val = 0;
      if(mController.getAButton())
      {
        val = 0.1;
      }
      else if(mController.getXButton())
      {
        val = 0.3;
      }
      else if(mController.getYButton())
      {
        val = 0.4;
      }
      else if(mController.getBButton())
      {
        val = 0.9;
      }
      //mLowerJointPIDController.setReference(val, ControlType.kPosition);

      mLowerJointPIDController.setReference(val, ControlType.kSmartMotion);
      SmartDashboard.putNumber("AppliedOutput", mLowerJointSpark.getAppliedOutput());

    SmartDashboard.putNumber("AbsoluteEncoder", mLowerJointAbsoluteEncoder.getPosition());
    SmartDashboard.putNumber("SetPoint", val);
    SmartDashboard.putNumber("RelativeEncoder", mLowerJointRelativeEncoder.getPosition());
*/
/*
double targetSpeedforPID = mController.getRightTriggerAxis() - mController.getLeftTriggerAxis();
mLowerJointPIDController.setReference(targetSpeedforPID, ControlType.kVelocity);
SmartDashboard.putNumber("LowerAbsoluteEncoderVelocity", mLowerJointAbsoluteEncoder.getVelocity());
SmartDashboard.putNumber("targetSpeedforPID", targetSpeedforPID);
SmartDashboard.putNumber("VelocityPIDError", targetSpeedforPID - mLowerJointAbsoluteEncoder.getVelocity());
*/
/*

        SmartDashboard.putBoolean("Forward Soft Limit Enabled",
                              mLowerJointSpark.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kForward));
    SmartDashboard.putBoolean("Reverse Soft Limit Enabled",
    mLowerJointSpark.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kReverse));                          
    SmartDashboard.putNumber("Forward Soft Limit",
    mLowerJointSpark.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
    SmartDashboard.putNumber("Reverse Soft Limit",
    mLowerJointSpark.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));
*/
    // mBottomIntakeMotor.set(speed);
    // mTopIntakeMotor.set(speed);

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
