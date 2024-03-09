// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import robosystems.ClimberArms;
import robosystems.ExtendoArm;
import robosystems.Intake;
import robosystems.Lights;
import robosystems.Shooter;
import robotcode.autonomous.AutonomousRoutine;
import robotcode.autonomous.RoutineFactory;
import states.ApproachingClimberControls;
import states.HoldingState;
import states.IntakingState;
import states.NextStateInfo;
import states.ShooterMode;
import states.ShootingState;
import states.StateMachine;
// import states.TestControls;
import states.JoystickControlsWithSwerve;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import universalSwerve.SwerveDrive;
import universalSwerve.controls.JoystickSwerveControls;
import universalSwerve.SwerveFactory;
import universalSwerve.components.WheelLabel;

import java.lang.Thread.State;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CorePigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkLimitSwitch;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;


import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  //MAIN COMPONENTS
  private SwerveDrive mSwerveDrive;
  private ExtendoArm mExtendoArm;
  private Intake mIntake;
  private Shooter mShooter;
  private Lights mLights;
  private LimelightManager mLimelightManager;
  private ClimberArms mClimberArms;

  private boolean mAnglerMotorAndGyroZeroingHasOccurred;
  private IntakingState mIntakingState;
  private HoldingState mHoldingState;
  private ShootingState mShootingState;

  private StateMachine mStateMachine;
  // private CorePigeon2 mPigeon2; 

  private XboxController mMainController;

  private JoystickSwerveControls mJoystickSwerveControls;
  //private

  

  
  private CANSparkMax mTrapRoller; ///8
  
  private final VelocityVoltage mVoltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
  


  public Robot()
  {
    super(0.005);//ovverride the loop timing... it was 20ms, set it to 10ms.
  }

  private TalonFX mLeftClimberMotor;
  private TalonFX mRightClimberMotor; 

  private Orchestra mOrchestra = new Orchestra();

  private void playStartupMusic()
  {  
     /*        
    var status = mOrchestra.loadMusic("sonic.chrp");
    if (!status.isOK()) 
    {
        
    }
    mOrchestra.play();
    */
  }

  private void RegisterSpeakers(java.util.List<TalonFX> pSpeakers)
  {
    for(int i = 0; i < pSpeakers.size(); i++)
    {
      RegisterSpeaker(pSpeakers.get(i));
    }
  }

  private void RegisterSpeaker(TalonFX pSpeaker)
  {

        AudioConfigs audioConfigs = new AudioConfigs();
        audioConfigs.AllowMusicDurDisable = true;
        audioConfigs.BeepOnBoot = false;
        audioConfigs.BeepOnConfig = false;
        pSpeaker.getConfigurator().apply(audioConfigs);

        mOrchestra.addInstrument(pSpeaker);
  }

  private CANSparkMax CreateTrapMotor(int pDeviceID, boolean pIsInverted)
  {
    CANSparkMax returnValue = new CANSparkMax(pDeviceID, MotorType.kBrushed);
    returnValue.setSmartCurrentLimit(39);
    returnValue.setIdleMode(IdleMode.kBrake);
    returnValue.setInverted(pIsInverted);
    returnValue.burnFlash();
    return returnValue;
  }




  private void CreateMotors()
  {
    mTrapRoller = CreateTrapMotor(8, false);          
  }


  private void CreateMainComponents()
  {
    mLimelightManager = new LimelightManager();
    mLights = new Lights();
    mIntake = new Intake();
    mShooter = new Shooter(mLimelightManager);
    mExtendoArm = new ExtendoArm();
    mClimberArms = new ClimberArms();
    mSwerveDrive = SwerveFactory.Create2024Swerve();
    

    mLeftClimberMotor = new TalonFX(11);
    mRightClimberMotor = new TalonFX(12);
    

    MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs();
    motorOutputConfig.NeutralMode = NeutralModeValue.Brake;
    mLeftClimberMotor.getConfigurator().apply(motorOutputConfig);
    mRightClimberMotor.getConfigurator().apply(motorOutputConfig);
    
        
  }

  private void CreateControls()
  {
    mMainController = new XboxController(0);
    mJoystickSwerveControls = new JoystickSwerveControls(mMainController);
  }

  
  private AutonomousRoutine mAutonomousRoutine;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    mMainController = new XboxController(0);

    
    // mPigeon2 = new CorePigeon2(22);

    mAnglerMotorAndGyroZeroingHasOccurred = false;
    CreateMainComponents();
    CreateControls();

    XboxController alternateController = new XboxController(1);
    
    JoystickControlsWithSwerve testControls = new JoystickControlsWithSwerve(mMainController, alternateController);
    mStateMachine = new StateMachine(testControls, mSwerveDrive, mIntake, mShooter, mExtendoArm, mClimberArms, mLights, mLimelightManager);
    // mIntakingState = new IntakingState(mSwerveDrive, mIntake, mShooter, testControls);
    // mHoldingState = new HoldingState(mSwerveDrive, mIntake, mShooter, testControls);
    // mShootingState = new ShootingState(mSwerveDrive, mIntake, mShooter, testControls);
    /*
    RegisterSpeakers(mSwerveDrive.GetSpeakers());
    RegisterSpeakers(mIntake.getSpeakers());
    RegisterSpeakers(mShooter.getSpeakers());

    playStartupMusic();
    */
    
    ApproachingClimberControls.setInstanceInformation(mMainController, alternateController);

    traj = Choreo.getTrajectory("NewPath");

    m_field.getObject("traj").setPoses(
      traj.getInitialPose(), traj.getFinalPose()
    );
    m_field.getObject("trajPoses").setPoses(
      traj.getPoses()
    );

    SmartDashboard.putData(m_field);


    CreateChoreoCommand();

    robotcode.autonomous.RoutineFactory routineFactory = new robotcode.autonomous.RoutineFactory(mSwerveDrive, mShooter, mIntake);
    mAutonomousRoutine = routineFactory.FourCloseRingAuto();

    



    //DataLogManager.start();
  }

  /*
  private void LogMotorAndSensorBasics()
  {
    SmartDashboard.putNumber("AnglerEncoder", mAnglerEncoder.getPosition());
    SmartDashboard.putNumber("AnglerOutputPercentage", mAngler.getAppliedOutput());
    SmartDashboard.putNumber("AnglerCurrentDraw", mAngler.getOutputCurrent());

    SmartDashboard.putNumber("ExtendoArmEncoder", mExtendoArmEncoder.getPosition());
    SmartDashboard.putNumber("ExtendoArmOutputPercentage", mExtendoArm.getAppliedOutput());
    SmartDashboard.putNumber("ExtendoArmCurrentDraw", mExtendoArm.getOutputCurrent());

    SmartDashboard.putNumber("HopperBreakBeam", mHopperBreakBeam.getVoltage());
    
  }
  */

  public void robotPeriodic()
  {
    mLimelightManager.calculateCameraPoseTargetSpace();
    double[] cameraResultsWest = mLimelightManager.getCameraPoseTargetSpaceForSpecificCamera(LimelightManager.WEST_CAMERA);
    double[] cameraResultsEast = mLimelightManager.getCameraPoseTargetSpaceForSpecificCamera(LimelightManager.EAST_CAMERA);
    
    if(cameraResultsWest != null)
    {
      SmartDashboard.putNumber("LL_West_JSON_0", cameraResultsWest[0]);
      SmartDashboard.putNumber("LL_West_JSON_2", cameraResultsWest[2]);
    }
    else
    {
      SmartDashboard.putNumber("LL_West_JSON_0",-1);
      SmartDashboard.putNumber("LL_West_JSON_2", -1);
    }

    if(cameraResultsEast != null)
    {
      SmartDashboard.putNumber("LL_East_JSON_0", cameraResultsEast[0]);
      SmartDashboard.putNumber("LL_East_JSON_2", cameraResultsEast[2]);
    }
    else
    {
      SmartDashboard.putNumber("LL_East_JSON_0",-1);
      SmartDashboard.putNumber("LL_East_JSON_2", -1);
    }
    
    if(cameraResultsWest != null && cameraResultsEast != null)
    {
      SmartDashboard.putNumber("LL_East_0_Delta", cameraResultsEast[0] - cameraResultsWest[0]);
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
  public void autonomousInit() 
  {

  
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (!mAnglerMotorAndGyroZeroingHasOccurred)
    {
      zeroAnglerEncoderAndGyro();
      return;
    }
    mAutonomousRoutine.Run();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    mSwerveDrive.EnableDiagnostics();
    mSwerveDrive.EnableDiagnostics(WheelLabel.NE);
    mSwerveDrive.EnableDiagnostics(WheelLabel.NW);
    mSwerveDrive.EnableDiagnostics(WheelLabel.SE);
    mSwerveDrive.EnableDiagnostics(WheelLabel.SW);



  }
  {
  
  }
  private void TestAngler()
  {    
      double setpoint = 60;//meaningless default
      boolean isButtonPressed = false;
      if(mMainController.getYButton())
      {
        setpoint = 100;
        isButtonPressed = true;
      }
      else if(mMainController.getBButton())
      {
        setpoint = 50;
        isButtonPressed = true;
      }
      else if(mMainController.getAButton())
      {
        setpoint = 30;
        isButtonPressed = true;
      }
      else if(mMainController.getXButton())
      {
        setpoint = 140;
        isButtonPressed = true;
      }

      if(isButtonPressed)
      {
        mShooter.setAngle(setpoint);
      }
      else
      {
        mShooter.stopAnglerMotor();
      }
  }

  private void testExtendoArm()
  {
      double setpoint = -1000;//meaningless default
      boolean isButtonPressed = false;
      if(mMainController.getYButton())
      {
        mExtendoArm.lowGoalExtension();
        isButtonPressed = true;
      }
      else if(mMainController.getBButton())
      {
        mExtendoArm.trapExtension();
        isButtonPressed = true;
      }
      else if(mMainController.getAButton())
      {
        mExtendoArm.retract();
        isButtonPressed = true;
      }
      else
      {
        mExtendoArm.stopMotor();
        
      }

  }

  private void testIntake()
  {
      double setpoint = -1000;//meaningless default
      boolean isButtonPressed = false;
      if(mMainController.getYButton())
      {
        mIntake.getBreakBeamStatus();
        isButtonPressed = true;
      }
      else if(mMainController.getBButton())
      {
        mIntake.setToFirstEjectingSpeed();
        isButtonPressed = true;
      }
      else if(mMainController.getAButton())
      {
        mIntake.setConveyorToBackupSpeed();
        isButtonPressed = true;
      }
      else
      {
        mIntake.stopMotors();
        
      }

  }

  private void testShooter()
  {
      double setpoint = -1000;//meaningless default
      boolean isButtonPressed = false;
      if(mMainController.getYButton())
      {
        mShooter.setSpeed(10, 10);
        isButtonPressed = true;
      }
      else if(mMainController.getBButton())
      {
        // mIntake.setToEjectingSpeed();
        // isButtonPressed = true;
      }
      else if(mMainController.getAButton())
      {
        // mIntake.setConveyorToBackupSpeed();
        // isButtonPressed = true;
      }
      else
      {
        mShooter.setSpeed(0, 0);        
      }

  }

  public NextStateInfo testIntakeState()
  {
    if (mMainController.getYButton())
    {
      return mIntakingState.Run();
    }
    else
    {
      mIntake.stopMotors();
      return null;
      
    }
  }

  public NextStateInfo testHoldingState()
  {
    if (mMainController.getYButton())
    {
      return mHoldingState.Run();
      
    }
    else
    {
      mIntake.stopMotors();
      mShooter.setSpeed(0, 0);
      return null;
      
    }
  }

  public NextStateInfo testShootingState()
  {
    if (mMainController.getYButton())
    {
      return mShootingState.Run();
    }
    else
    {
      mShooter.setSpeed(0, 0);
      mIntake.stopMotors();
      return null;
    }
  }

  public void testDriveTrainAndIntake()
  {
    mSwerveDrive.Run(mJoystickSwerveControls);
    if (mMainController.getRightBumper())
    {
      mIntake.setToNormalIntakingSpeed();
    }
    else
    {
      mIntake.stopMotors();
    }
  }

  private double lastRecordedX = 0; 

  private void logLimelightInfo(String pCameraName, double[] camerapose_targetspace)
  {

    
   if (camerapose_targetspace != null)
    {
      SmartDashboard.putNumber(pCameraName + "Lime: X change since last TS", camerapose_targetspace[0] - lastRecordedX);
      lastRecordedX = camerapose_targetspace[0];
      SmartDashboard.putNumber(pCameraName + "X displacement", camerapose_targetspace[0]);
      SmartDashboard.putNumber(pCameraName + "Y displacement", camerapose_targetspace[1]);
      SmartDashboard.putNumber(pCameraName + "Z displacement", camerapose_targetspace[2]);

      SmartDashboard.putNumber(pCameraName + "Roll displacement", camerapose_targetspace[3]);
      SmartDashboard.putNumber(pCameraName + "Pitch displacement", camerapose_targetspace[4]);
      SmartDashboard.putNumber(pCameraName + "Yaw displacement", camerapose_targetspace[5]);
      SmartDashboard.putNumber(pCameraName + "Distance from tag", Math.abs(camerapose_targetspace[0] - (-0.29)));
      SmartDashboard.putBoolean(pCameraName + "Can See Tag", true);
    }
    else
    {
      SmartDashboard.putNumber(pCameraName + "X displacement", -1);
      SmartDashboard.putNumber(pCameraName + "Y displacement", -1);
      SmartDashboard.putNumber(pCameraName + "Z displacement", -1);
      SmartDashboard.putNumber(pCameraName + "Roll displacement", -1);
      SmartDashboard.putNumber(pCameraName + "Pitch displacement", -1);
      SmartDashboard.putNumber(pCameraName + "Yaw displacement", -1);
      SmartDashboard.putBoolean(pCameraName + "Can See Tag", false);
      
    }
  }

  public void logLimeLightInfo()
  {

    try
    {
      SmartDashboard.putNumber("LimeLightWestMinusEast", mLimelightManager.getCameraPoseTargetSpaceForSpecificCamera(LimelightManager.WEST_CAMERA)[0]  - mLimelightManager.getCameraPoseTargetSpaceForSpecificCamera(LimelightManager.EAST_CAMERA)[0] );
    }
    catch(Exception e)
      {

      }
    

    logLimelightInfo("East-", mLimelightManager.getCameraPoseTargetSpaceForSpecificCamera(LimelightManager.EAST_CAMERA));
    logLimelightInfo("West-", mLimelightManager.getCameraPoseTargetSpaceForSpecificCamera(LimelightManager.WEST_CAMERA));
    logLimelightInfo("Combined-", mLimelightManager.getCameraPoseTargetSpace());
   
  }

  public void zeroAnglerEncoderAndGyro()
  {
    SmartDashboard.putBoolean("Limit Reached", mShooter.mAnglerHardReverseLimitSwitchisPressed());
    // SmartDashboard.putNumber("Angler Pos", mAnglerMotor.getEncoder().getPosition());
    
    if (!mAnglerMotorAndGyroZeroingHasOccurred)
    {
      mShooter.setAnglerSpeed(-0.05);
      
      if (mShooter.isAnglerHardReverseLimitSwitchPressed())
      {
        mSwerveDrive.SetGyroscopeCurrentAngle(0); // remember to set this back to 0 for real game.  180 is for testing where you can ssee the foal.
    
        mAnglerMotorAndGyroZeroingHasOccurred = true;
        mShooter.setAnglerSpeed(0);
        mShooter.zeroAnglerMotorInformation();
        Constants.setLowGoalRotation();
        mSwerveDrive.InitializeOdometry();
      }
    }
    else
    {
      SmartDashboard.putString("Status","Not Here");
    }
  }


  

  private long mLastEntryTime;
  private double mPosition = -800;
  public void teleopPeriodic()
  {
    logLimeLightInfo();
    // if (mMainController.getAButton())
    // {
    //   mClimberArms.setSpeed(0.2);
    // }
    // else if (mMainController.getBButton())
    // {
    //   mClimberArms.setSpeed(-0.2);
    // }
    // else
    // {
    //   mClimberArms.setSpeed(0);
    // }

    // mSwerveDrive.Run(ApproachingClimberControls.Instance);
    // boolean mFinishedRetracting = mClimberArms.retract();
      
    // SmartDashboard.putBoolean("Climber: mFinishedRetracting", mFinishedRetracting);
    // SmartDashboard.putNumber("Right Climber Position", mClimberArms.getRightPosition());


    mStateMachine.Run();

    // mSwerveDrive.Run(ApproachingClimberControls.Instance);
    // boolean mFinishedRetracting = mClimberArms.retract();
    // // boolean mFinishedExtending = mClimberArms.extend();
    // mShooter.goToTrapShootAngle();

    // mShooter.goToTrapShootAngle();


    /* 
    // mSwerveDrive.Run(mJoystickSwerveControls);
    long entryTime = System.currentTimeMillis();
    SmartDashboard.putNumber("TimeBetweenCallsToTeleopPeriodic", mLastEntryTime - entryTime);
    mShooter.logShooterAngle();
    mLastEntryTime = entryTime;
   
 
    
    logLimeLightInfo();
      mSwerveDrive.LogDiagnostics();
    if (!mAnglerMotorAndGyroZeroingHasOccurred)
    {
      zeroAnglerEncoderAndGyro();
      return;
    }
    
    if (mSwerveDrive.getLastRequestedChassisSpeeds() != null)
    {
      SmartDashboard.putNumber("Last Rq Chassis Speeds VY", mSwerveDrive.getLastRequestedChassisSpeeds().vyMetersPerSecond);
    }
    else
    {
      SmartDashboard.putNumber("Last Rq Chassis Speeds VY", -1);
    }
    mStateMachine.Run();
    
    long exitTime = System.currentTimeMillis();

    SmartDashboard.putNumber("TimeOfTelopPeriodic", exitTime - entryTime);
    */
    
  
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() 
  {


  }


  private void testShooterMotors()
  {
    List<TalonFX> shooterMotors =  mShooter.getSpeakers();
    shooterMotors.get(0).set(0.5);
    shooterMotors.get(1).set(0.5);
    mShooter.logShooterInformation();
  }
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() 
  {
    if (mMainController.getAButton())
    {
      mClimberArms.setLeftSpeed(0.2);
    }
    else if (mMainController.getBButton())
    {
      mClimberArms.setLeftSpeed(-0.2);
    }
    else if (mMainController.getXButton())
    {
      mClimberArms.setRightSpeed(0.2);
    }
    else if (mMainController.getYButton())
    {
      mClimberArms.setRightSpeed(-0.2);
    }
    else
    {
      mClimberArms.setSpeed(0);
    }

    if (mMainController.getRightBumper())
    {
      mShooter.setAnglerSpeed(-0.2);
    }
    else
    {
      mShooter.setAnglerSpeed(0);
    }
    // testShooterMotors();
    return;
  }
    // if (!mAnglerMotorAndGyroZeroingHasOccurred)
    // {
    //   zeroAnglerEncoderAndGyro();
    //   return;
    // }
    // mSwerveDrive.Run(mJoystickSwerveControls);
   public void fake()
   {
    mIntake.setToNormalIntakingSpeed();
    // testChoreo1();
    if (mMainController.getAButton())
    {
      mClimberArms.setSpeed(0.2);
    }
    else if (mMainController.getBButton())
    {
      mClimberArms.setSpeed(-0.2);
    }
    else
    {
      mClimberArms.setSpeed(0);
    }

    if (mMainController.getXButton())
    {
      mShooter.setAnglerSpeed(-0.2);
    }
    else if (mMainController.getYButton())
    {
      mShooter.setAnglerSpeed(0.2);
    }
    else
    {
      mShooter.setAnglerSpeed(0);
    }
    
    //testChoreo1();

    //testLowGoal();
    //testTrapIntake();

 



  }

  public boolean ShouldMirrorTrajectories()
  {
    return false;
  }

  private Command mChoreoCommand;
  private void CreateChoreoCommand()
  {
      var thetaController = new PIDController(10, 0, 0);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      mChoreoCommand = Choreo.choreoSwerveCommand(
        traj, // Choreo trajectory from above
        mSwerveDrive::getPose, // A function that returns the current field-relative pose of the robot: your
                               // wheel or vision odometry
        new PIDController(0.1, 0.0, 0.0), // PIDController for field-relative X
                                                                                   // translation (input: X error in meters,
                                                                                   // output: m/s).
        new PIDController(0.1, 0.0, 0.0), // PIDController for field-relative Y
                                                                                   // translation (input: Y error in meters,
                                                                                   // output: m/s).
        thetaController, // PID constants to correct for rotation
                         // error

        
        /* this is what the choreo documentation says, but I think we think about things quite different from them in terms of X/Y
        (ChassisSpeeds speeds) -> m_robotDrive.drive( // needs to be robot-relative
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            false),
        */
        (ChassisSpeeds speeds) -> mSwerveDrive.Run( // needs to be robot-relative
          -speeds.vyMetersPerSecond,    
          speeds.vxMetersPerSecond,            
          speeds.omegaRadiansPerSecond),
        this::ShouldMirrorTrajectories ,       
        new Subsystem[0]
        //, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
        //I suspect because I am calling the Run command above I don't need to spcity this? mSwerveDrive // The subsystem(s) to require, typically your drive subsystem only
    );
  }

  


  private Field2d m_field = new Field2d();
  private ChoreoTrajectory traj;
  private boolean mHasStartedTrajectory = false;
  private Timer mTimer = new Timer();
  private void testChoreo1()
  {

    
    if(mMainController.getAButton())
    {
      if(!mHasStartedTrajectory)
      {
        //I guess this assumes that you're starting from the right odometry?
        //Not quite sure.  Only should do this at the start of the trajectory, presumably.
        
        mHasStartedTrajectory = true;
        mTimer = new Timer();
        mTimer.start();
        mChoreoCommand.initialize();
        mSwerveDrive.ResetDistanceTravelled();
        mSwerveDrive.ResetOdometry(traj.getInitialPose());         
        
      }
      mSwerveDrive.UpdateOdometry();
      mChoreoCommand.execute();
      ChoreoTrajectoryState desiredCurrentState = traj.sample(mTimer.get());
      SmartDashboard.putNumber("Choreo_desiredCurrentState.x", desiredCurrentState.x);
      SmartDashboard.putNumber("Choreo_ desiredCurrentState.y", desiredCurrentState.y);
      SmartDashboard.putNumber("Choreo_desiredCurrentState.heading", desiredCurrentState.heading);
      SmartDashboard.putNumber("Choreo_desiredCurrentState.velocityX", desiredCurrentState.velocityX);
      SmartDashboard.putNumber("Choreo_desiredCurrentState.velocityY", desiredCurrentState.velocityY);
      SmartDashboard.putNumber("Choreo_desiredCurrentState.angularVelocity", desiredCurrentState.angularVelocity);

      SmartDashboard.putNumber("mSwerveDrive.getPose().getY()", mSwerveDrive.getPose().getY());
      SmartDashboard.putNumber("mSwerveDrive.getPose().getX()", mSwerveDrive.getPose().getX());

      
    }
    else
    {
      //mChoreoCommand.cancel();
      mHasStartedTrajectory = false;
      mSwerveDrive.StopEverything();
    }


  

  }


  private void testTrapIntake()
  {
     int pov = mMainController.getPOV(); 
    SmartDashboard.putNumber("POV", pov);
    
    if(Math.abs(pov - 0.0) < 0.1)
    {
      mExtendoTargetPosition -= 1.0;
    } 
    else if(Math.abs(pov - 180.0) < 0.1)
    {
      mExtendoTargetPosition += 1.0;
    }

    mExtendoArm.goToPosition(mExtendoTargetPosition);
    SmartDashboard.putNumber("ExtendoTarget", mExtendoTargetPosition);
    SmartDashboard.putNumber("ExtendoCurrent", mExtendoArm.getCurrentDraw());
    SmartDashboard.putNumber("ExtendoEncoder", mExtendoArm.getEncoderReading());

    mShooter.setAnglerSpeed((mMainController.getLeftTriggerAxis() - mMainController.getRightTriggerAxis()) / 10.0);

    if(mMainController.getRightBumper())
    {
      mIntake.TestSetTrapIntakeSpeed(0.75);
    }
    else
    {
       mIntake.TestSetTrapIntakeSpeed(0);
    }


      
    if (mMainController.getYButton())
    {
      mRightClimberMotor.set(0.2);
    }
    else if (mMainController.getXButton())
    {
      mRightClimberMotor.set(-0.2);
    }
    else
    {
      mRightClimberMotor.set(0);
    }

    if (mMainController.getBButton())
    {
      mLeftClimberMotor.set(-0.2);
    }
    else if (mMainController.getAButton())
    {
      mLeftClimberMotor.set(0.2);
    }
    else
    {
      mLeftClimberMotor.set(0);
    }

  }

  private double mTestLowGoalBottomShooterSpeed = 26.4;
  private double mTestLowGoalTopShooterSpeed = 7.8;
  private double mExtendoTargetPosition = 0;
  private void testLowGoal()
  {
    double speedDelta = 0.1;
    if(mMainController.getAButton())
    {
      mTestLowGoalBottomShooterSpeed -= speedDelta;
    }
    else if(mMainController.getBButton())
    {
      mTestLowGoalBottomShooterSpeed += speedDelta;
    }


    if(mMainController.getXButton())
    {
      mTestLowGoalTopShooterSpeed -= speedDelta;
    }
    else if(mMainController.getYButton())
    {
      mTestLowGoalTopShooterSpeed += speedDelta;
    }

    mShooter.setSpeed(mTestLowGoalTopShooterSpeed, mTestLowGoalBottomShooterSpeed);
    mShooter.setAnglerSpeed((mMainController.getLeftTriggerAxis() - mMainController.getRightTriggerAxis()) / 10.0);

    SmartDashboard.putNumber("mTestLowGoalTopShooterSpeed", mTestLowGoalTopShooterSpeed);
    SmartDashboard.putNumber("mTestLowGoalBottomShooterSpeed", mTestLowGoalBottomShooterSpeed);
    SmartDashboard.putNumber("anglerAngle", mShooter.GetAnglerEncoderReading());

    if(mMainController.getRightBumper())
    {
      mIntake.setToLaunchingNoteIntoTheShooterSpeed();
    }
    else
    {
      //mIntake.setToHoldingSpeed();
      mIntake.stopMotors();
    }

    int pov = mMainController.getPOV(); 
    SmartDashboard.putNumber("POV", pov);
    
    if(Math.abs(pov - 0.0) < 0.1)
    {
      mExtendoTargetPosition -= 1.0;
    } 
    else if(Math.abs(pov - 180.0) < 0.1)
    {
      mExtendoTargetPosition += 1.0;
    }

    mExtendoArm.goToPosition(mExtendoTargetPosition);
    SmartDashboard.putNumber("ExtendoTarget", mExtendoTargetPosition);
    SmartDashboard.putNumber("ExtendoCurrent", mExtendoArm.getCurrentDraw());
    SmartDashboard.putNumber("ExtendoEncoder", mExtendoArm.getEncoderReading());

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
