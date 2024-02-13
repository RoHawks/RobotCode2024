// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import universalSwerve.SwerveDrive;
import universalSwerve.controls.JoystickSwerveControls;
import universalSwerve.SwerveFactory;

import java.lang.Thread.State;
import java.util.Arrays;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class RobotTestsFrom20240210 extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private SwerveDrive mSwerveDrive;
  private CANSparkMax mRollerMotorTop; 
  private CANSparkMax mShooterMotorTop;
  private CANSparkMax mIntake;
  private CANSparkFlex mRollerMotorBottom; 
  private CANSparkMax mShooterMotorBottom; 
  private CANSparkMax mAnglerMotor;
  private CANSparkMax mExtendoArmMotor; 
  private XboxController mMainController;

  private boolean mAToggle;
  private boolean mShooterOn; 
  
  private boolean mBToggle;
  private boolean mRollerOn;

  private double mStoredTime;

  
  private DigitalInput mBreakBeam;

  private enum StateForTesting {kIntaking, kWaitForRollersToStop, kBackup, kShoot, kOff};
  private StateForTesting mStateForTesting;

  //private SwerveDrive mSwerveDrive;
  private JoystickSwerveControls mJoystickSwerveControls;
  //private

  private RelativeEncoder mExtendoArmEncoder;

  private DigitalOutput mBreakBeamTransmitter;



  TalonFX mSESwerveDrive; //62
  TalonFX mNESwerveDrive; //1
  TalonFX mNWSwerveDrive; //18
  TalonFX mSWSwerveDrive; //19

  CANSparkMax mSESwerveTurn; //2
  CANSparkMax mNESwerveTurn; //3
  CANSparkMax mNWSwerveTurn; //16
  CANSparkMax mSWSwerveTurn; //17

  TalonFX mTopShooterRoller; //4
  TalonFX mBottomShooterRoller; //5
  TalonFX mBottomIntakeConveyor; //6
  TalonFX mTopIntakeConveyor; //13


  CANSparkMax mExtendoArms; //7
  CANSparkMax mExtendableRoller; ///8
  CANSparkFlex mAngler; //14
  CANSparkMax mFrontIntakeRoller; //15
  
  private final VelocityVoltage mVoltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
  
  public void quickMotorInit()
  {
    mSESwerveDrive = new TalonFX(62); //62
    mNESwerveDrive = new TalonFX(1); //1
    mNWSwerveDrive = new TalonFX(18); //18
    mSWSwerveDrive = new TalonFX(19);; //19

    mSESwerveTurn = new CANSparkMax(2, MotorType.kBrushless); //2
    mNESwerveTurn = new CANSparkMax(3, MotorType.kBrushless); //3
    mNWSwerveTurn = new CANSparkMax(16, MotorType.kBrushless); //16
    mSWSwerveTurn = new CANSparkMax(17, MotorType.kBrushless); //17

    configureSparkMax(mSESwerveTurn);
    configureSparkMax(mNESwerveTurn);
    configureSparkMax(mNWSwerveTurn);
    configureSparkMax(mSWSwerveTurn);

    mTopShooterRoller = new TalonFX(4); //4
    mBottomShooterRoller = new TalonFX(5); //5
    mBottomIntakeConveyor = new TalonFX(6);; //6
    mTopIntakeConveyor = new TalonFX(13);; //13


    mExtendoArms = new CANSparkMax(7, MotorType.kBrushed); //7
    mExtendableRoller = new CANSparkMax(8, MotorType.kBrushless);  ///8
    mAngler = new CANSparkFlex(14, MotorType.kBrushless); //14
    mFrontIntakeRoller = new CANSparkMax(15, MotorType.kBrushed); ; //15
    configureSparkMax(mExtendoArms);
    configureSparkMax(mExtendableRoller);
   
    mAngler.setClosedLoopRampRate(0.2);
    mAngler.setSmartCurrentLimit(20);
    mAngler.setIdleMode(IdleMode.kBrake);
    configureSparkMax(mFrontIntakeRoller);

    mTopShooterRoller.setInverted(true);
    mBottomShooterRoller.setInverted(false);
    mFrontIntakeRoller.setInverted(true);
    TalonFXConfiguration topShooterConfig = new TalonFXConfiguration();
    //topShooterConfig.Slot0.kS = 1.0/10.0;
    topShooterConfig.Slot0.kP = 0.2;
    topShooterConfig.Slot0.kI = 0;
    topShooterConfig.Slot0.kD = 0;
    topShooterConfig.Slot0.kV = 0.137;
    
    mTopShooterRoller.getConfigurator().apply(topShooterConfig);
   
     TalonFXConfiguration bottomShooterConfig = new TalonFXConfiguration();
    //topShooterConfig.Slot0.kS = 1.0/10.0;
    bottomShooterConfig.Slot0.kP = 0.2;
    bottomShooterConfig.Slot0.kI = 0;
    bottomShooterConfig.Slot0.kD = 0;
    bottomShooterConfig.Slot0.kV = 0.137;

    mBottomShooterRoller.getConfigurator().apply(bottomShooterConfig);


  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // mShooterMotorTop = new CANSparkMax(13, MotorType.kBrushless);
    // mShooterMotorTop.setSmartCurrentLimit(40);
    // mShooterMotorTop.setClosedLoopR  ampRate(0.2);
    // mShooterMotorTop.setIdleMode(IdleMode.kCoast);

    // mShooterMotorBottom = new CANSparkMax(54, MotorType.kBrushless);
    // mShooterMotorBottom.setSmartCurrentLimit(40);
    // mShooterMotorBottom.setClosedLoopRampRate(0.2);
    // mShooterMotorBottom.setIdleMode(IdleMode.kCoast);

    // mRollerMotorTop = new CANSparkMax(4, MotorType.kBrushless);
    // mRollerMotorTop.setSmartCurrentLimit(20);
    // mRollerMotorTop.setClosedLoopRampRate(0.2);
    // mRollerMotorTop.setIdleMode(IdleMode.kCoast);

    // mIntake = new CANSparkMax(59, MotorType.kBrushless);
    // mIntake.setSmartCurrentLimit(20);
    // mIntake.setClosedLoopRampRate(0.2);
    // mIntake.setIdleMode(IdleMode.kCoast);
    // mIntake.enableSoftLimit(SoftLimitDirection.kForward, false);
    // mIntake.enableSoftLimit(SoftLimitDirection.kReverse, false);



    // //VORTEX
    // mRollerMotorBottom = new CANSparkFlex(37, MotorType.kBrushless);
    // mRollerMotorBottom.setSmartCurrentLimit(38);
    // mRollerMotorBottom.setClosedLoopRampRate(0.2);
    
    // mAnglerMotor = new CANSparkMax(47, MotorType.kBrushless);
    // mAnglerMotor.setSmartCurrentLimit(20);
    // mAnglerMotor.setClosedLoopRampRate(0.2);
    // mAnglerMotor.setIdleMode(IdleMode.kBrake);


    // mExtendoArmMotor = new CANSparkMax(26, MotorType.kBrushed);
    // mExtendoArmMotor.restoreFactoryDefaults();
    // mExtendoArmMotor.setSmartCurrentLimit(40);
    // mExtendoArmMotor.setClosedLoopRampRate(0.01);
    // mExtendoArmMotor.setIdleMode(IdleMode.kBrake);
    // mExtendoArmMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    // mExtendoArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    // mExtendoArmEncoder = mExtendoArmMotor.getEncoder(Type.kQuadrature, 1);

    mMainController = new XboxController(0);
    // mJoystickSwerveControls = new JoystickSwerveControls(mMainController);
    // double shooterP;
    // double shooterI;
    // double shooterD;

    // shooterP = 1;
    // shooterI = 0;
    // shooterD = 0;

    // mShooterMotorBottom.getPIDController().setP(shooterP);
    // mShooterMotorBottom.getPIDController().setI(shooterI);
    // mShooterMotorBottom.getPIDController().setD(shooterD);

    // mBreakBeam = new DigitalInput(5); // CHANGE THIS LATER
    // mBreakBeamTransmitter = new DigitalOutput(6);
    // mBreakBeamTransmitter.set(true);
    // // mShooterMotor.getPIDController().setP(1);
    // // mShooterMotor.getPIDController().setI(1);
    // // mShooterMotor.getPIDController().setD(0);
    
    // // mShooterMotor.getPIDController().setFeedbackDevice(
    // //   mShooterMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle));

    // mSwerveDrive = SwerveFactory.Create2022Swerve();

    // mAToggle = false;
    // mBToggle = false;
    // mRollerOn = false;
    // mShooterOn = false;


    quickMotorInit();
    //mSwerveDrive = SwerveFactory.Create2022Swerve();
  }

  public void robotPeriodic()
  {

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  
  public void robotPeriodicPracticeShooting() 
  {

    
  
    // //mSwerveDrive.Run(new JoystickSwerveCont rols(mMainController));

    
    // SmartDashboard.putNumber("BottomRollerCurrent", mRollerMotorBottom.getOutputCurrent());


    // double shooterSpeed = 0.95;
    // //double shooterSpeed = 0.5;//For low goal

    // //mShooterMotorTop.getPIDController().setReference(shooterSpeed, null, 0, shooterSpeed, null)
    // if (mMainController.getBButton())
    // {
    //   mShooterMotorTop.set(-1 * shooterSpeed);
    //   mShooterMotorBottom.set(shooterSpeed);
    //   SmartDashboard.putNumber("Top Shooter Motor Speed", mShooterMotorTop.getEncoder().getVelocity() *-1.0);
    //   SmartDashboard.putNumber("Bottom Shooter Motor Speed", mShooterMotorBottom.getEncoder().getVelocity());
    // }
    // else
    // {
    //   mShooterMotorTop.set(0);
    //   mShooterMotorBottom.set(0);
    // }


    // if (mMainController.getRightBumper()) 
    // { 

    //   mRollerMotorTop.set(1);
    //   mRollerMotorBottom.set(1);
    //   mIntake.set(-1.0);
    // }
    // else
    // {

    //   mRollerMotorTop.set(0);
    //   mRollerMotorBottom.set(0);
    //   mIntake.set(0);
    // }

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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    mStateForTesting = StateForTesting.kOff;
  }


  public void teleopTestingFebTenthInit()
  {
    /*
    {
    if (mMainController.getAButton()){ 
      mSESwerveDrive.set(0.3);
    }
    else {
      mSESwerveDrive.set(0);
    } 

    if (mMainController.getBButton()) 
    { 
      mNESwerveDrive.set(0.3);
    }
    else 
    {
      mNESwerveDrive.set(0);
    }
    
    if (mMainController.getXButton()) 
    { 
      mNWSwerveDrive.set(0.3);
    }
    else 
    {
    mNWSwerveDrive.set(0);
    }
    
    if (mMainController.getYButton()) 
    { 
      mSWSwerveDrive.set(0.3);
    }
    else 
    {
    mSWSwerveDrive.set(0);
    }
    
    if (mMainController.getLeftBumper()) 
    { 
      mSESwerveTurn.set(0.3); 
    }
    else 
    {
      mSESwerveTurn.set(0); 
    }
    
    if (mMainController.getRightBumper()) 
    { 
      mNESwerveTurn.set(0.3); 
    }
    else 
    {
      mNESwerveTurn.set(0); 
    }
  }
    */
    // mTopShooterRoller = new TalonFX(4); //4
    // mBottomShooterRoller = new TalonFX(5); //5
    // mBottomIntakeConveyor = new TalonFX(6);; //6
    // mTopIntakeConveyor = new TalonFX(13);; //13


    // mExtendoArms = new CANSparkMax(7, MotorType.kBrushed); //7
    // mExtendableRoller = new CANSparkMax(8, MotorType.kBrushless);  ///8
    // mAngler = new CANSparkFlex(14, MotorType.kBrushless); //14
    // mFrontIntakeRoller = new CANSparkMax(15, MotorType.kBrushless); ; //15

    // Front Intake Roller: Inverted
    // Top Shooter Roller: Inverted
    // Bottom Shooter Roller: Inverted
    // if (mMainController.getAButton()) 
    // {  
    //   mTopShooterRoller.setControl(mVoltageVelocity.withVelocity(-46));
      
    //   mBottomShooterRoller.setControl(mVoltageVelocity.withVelocity(46 * 3.0/4.0));
      
    // }
    // else 
    // {
    //   mTopShooterRoller.set(0);
    //   mBottomShooterRoller.set(0);
    // }

    SmartDashboard.putNumber("Top Shooter Speed", -mTopShooterRoller.getVelocity().getValue());
    SmartDashboard.putNumber("Bottom Shooter Speed", mBottomShooterRoller.getVelocity().getValue());

  
    
    // if (mMainController.getXButton()) 
    // { 
    //   mBottomIntakeConveyor.set(1);
    //   mTopIntakeConveyor.set(1);
    //   mFrontIntakeRoller.set(1); 
    // }
    // else 
    // {
    //   mBottomIntakeConveyor.set(0);
    //   mTopIntakeConveyor.set(0);
    //   mFrontIntakeRoller.set(0); 
    // }
    

    if (mMainController.getAButton())
    {
      mExtendoArms.set(0.5);
    }
    else if (mMainController.getBButton())
    {
      mExtendoArms.set(-0.5);
    }
    else
    {
      mExtendoArms.set(0);
    }


  }

  public void configureSparkMax(CANSparkMax pSparkMax)
  {
    pSparkMax.setSmartCurrentLimit(20);
    pSparkMax.setClosedLoopRampRate(0.2);
  }

  public void configureTalonOrFalcon(TalonFX pTalon)
  {
    // pTalon.set()
    // pSparkMax.setClosedLoopRampRate(0.2);
  }

  public void teleopTestingFebTenthPeriodic()
  {
    

  }

  public void teleopPeriodic()
  {

    // SmartDashboard.putNumber("ExtendoArm Position", mExtendoArmEncoder.getPosition());
    // SmartDashboard.putBoolean("Breakbeam Boolean", mBreakBeam.get());
    // SmartDashboard.putNumber("Top Shooter Wheel Velocity", mShooterMotorTop.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Top Roller Position", mRollerMotorTop.getEncoder().getPosition());
    // SmartDashboard.putNumber("Top Roller Velocity", mRollerMotorTop.getEncoder().getVelocity());
    
    // SmartDashboard.putNumber("Bottom Shooter Wheel Velocity", mShooterMotorBottom.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Bottom Roller Position", mRollerMotorBottom.getEncoder().getPosition());
    // SmartDashboard.putNumber("Bottom Roller Velocity", mRollerMotorBottom.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Current Of ExtendoArm Motor", mExtendoArmMotor.getOutputCurrent());
    
    // SmartDashboard.putNumber("Time", System.currentTimeMillis());
    

    teleopTestingFebTenthInit();
    //  teleoperatedTestStatesJan28();
    //robotPeriodicPracticeShooting();
  }
  /** This function is called periodically during operator control. */
  


  public void teleoperatedTestSimpleJan28()
  {
    
    if (mMainController.getXButton())
    {
      setShooterSpeed(0.50);
    }
    else
    {
      setShooterSpeed(0);
    }

    if (mMainController.getYButton())
    {
      setRollerSpeed(1);
    }
    else
    {
      setRollerSpeed(0);
    }

    if (mMainController.getBButton())
    {
      mExtendoArmMotor.set(-1);
    }
    else if(mMainController.getAButton())
    {
      mExtendoArmMotor.set(1);
    }
    else
    {
      mExtendoArmMotor.set(0);
    }

    
  }

  private double mRollerMotorTopPosition;
  public void teleoperatedTestStatesJan28()
  {
    
    
    
    if (mMainController.getXButton())
    {
      mStateForTesting = StateForTesting.kOff;
    }
    if (mMainController.getYButton())
    {
      mStateForTesting = StateForTesting.kIntaking;
    }
    SmartDashboard.putString("State", mStateForTesting.name());
    // if (mMainController.getBButton())
    // {
    //   mStateForTesting = StateForTesting.kBackup;
    // }
    // if (mMainController.getAButton())
    // {
    //   mStateForTesting = StateForTesting.kShoot;
    // }

    double test_speed = 0.3;
    
    if (mStateForTesting == StateForTesting.kOff)
    {
      setShooterSpeed(0);
      setRollerSpeed(0);
      setIntakeSpeed(0);
    }
    if (mStateForTesting == StateForTesting.kIntaking)
    {
      setShooterSpeed(0);
      setRollerSpeed(test_speed*2);
      setIntakeSpeed(test_speed*2);

      if (!mBreakBeam.get())
      {
        mStateForTesting = StateForTesting.kBackup;
        mRollerMotorTopPosition = mRollerMotorTop.getEncoder().getPosition();
        SmartDashboard.putNumber("Position before Backup RMT", mRollerMotorTopPosition);
      
      }
    }

    


    if (mStateForTesting == StateForTesting.kBackup)
    {
      SmartDashboard.putNumber("Difference in Distance RMT", mRollerMotorTop.getEncoder().getPosition() - mRollerMotorTopPosition);
      SmartDashboard.putNumber("Position before Backup RMT", mRollerMotorTopPosition);
      SmartDashboard.putNumber("Position RMT", mRollerMotorTop.getEncoder().getPosition());
      
      if (mRollerMotorTop.getEncoder().getPosition() - mRollerMotorTopPosition > -1)
      {
        SmartDashboard.putString("Here", "nosir");
        setRollerSpeed(-0.1);
      }
      else
      {
        SmartDashboard.putString("Here", "yessir");
        setRollerSpeed(0.0);
        setIntakeSpeed(0.0);
        if (mMainController.getAButton())
        {
          mStateForTesting = StateForTesting.kShoot;
          mStoredTime = -1; // value that wouldn't occur through other means in the program to know 
                            // that it's unset
        }
      }
    }
    if (mStateForTesting == StateForTesting.kShoot)
    {
      setShooterSpeed(test_speed);
      if (mShooterMotorTop.getEncoder().getVelocity() < -1300)
      {
        setRollerSpeed(test_speed);
        if (mStoredTime < 0) // bc it's -1 
        {
          mStoredTime = System.currentTimeMillis();
        }
        // Store time 
      }

      // When time elapsed greater than 3000 ms
      if (System.currentTimeMillis() > mStoredTime + 3000 && mStoredTime > 0) 
        mStateForTesting = StateForTesting.kIntaking;
      
    }
  }


  private void setShooterSpeed(double pSpeed)
  {
    mShooterMotorBottom.set(pSpeed);
    mShooterMotorTop.set(-pSpeed);
  }
  private void setRollerSpeed(double pSpeed)
  {
    mRollerMotorBottom.set(pSpeed);
    mRollerMotorTop.set(pSpeed);
  }

  private void setIntakeSpeed(double pSpeed)
  {
    mIntake.set(-pSpeed);
  }

  public void teleopPeriodicTest1()
  {
    mSwerveDrive.Run(mJoystickSwerveControls);



    double shooterSpeed = 0.3;
    //double shooterSpeed = 0.5;//For low goal

    //mShooterMotorTop.getPIDController().setReference(shooterSpeed, null, 0, shooterSpeed, null)
    if (mMainController.getLeftBumper())
    {
      mShooterMotorTop.set(-1 * shooterSpeed);
      mShooterMotorBottom.set(shooterSpeed);
      SmartDashboard.putNumber("Top Shooter Motor Speed", mShooterMotorTop.getEncoder().getVelocity() *-1.0);
      SmartDashboard.putNumber("Bottom Shooter Motor Speed", mShooterMotorBottom.getEncoder().getVelocity());
    }
    else
    {
      mShooterMotorTop.set(0);
      mShooterMotorBottom.set(0);
    }

    

    if (mMainController.getRightBumper()) 
    { 

      mRollerMotorTop.set(1);
      mRollerMotorBottom.set(1);
      mIntake.set(-1.0 * 1);
    }
    else
    {

      mRollerMotorTop.set(0);
      mRollerMotorBottom.set(0);
      mIntake.set(0);
    }



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
