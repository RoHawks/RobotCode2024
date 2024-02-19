// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robosystems.ExtendoArm;
import robosystems.Intake;
import robosystems.Shooter;
import states.HoldingState;
import states.IntakingState;
import states.NextStateInfo;
import states.ShootingState;
import states.StateMachine;
import states.TestControls;
import states.TestControlsWithSwerve;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import universalSwerve.SwerveDrive;
import universalSwerve.controls.JoystickSwerveControls;
import universalSwerve.SwerveFactory;
import universalSwerve.components.WheelLabel;

import java.lang.Thread.State;
import java.util.Arrays;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CorePigeon2;
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
    mIntake = new Intake();
    mShooter = new Shooter();
    mExtendoArm = new ExtendoArm();
    mSwerveDrive = SwerveFactory.Create2024Swerve();
  }

  private void CreateControls()
  {
    mMainController = new XboxController(0);
    mJoystickSwerveControls = new JoystickSwerveControls(mMainController);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  CANSparkFlex mAnglerMotor; 
  @Override
  public void robotInit() {
    mMainController = new XboxController(0);
    // mPigeon2 = new CorePigeon2(22);

    mAnglerMotorAndGyroZeroingHasOccurred = false;
    CreateMainComponents();
    CreateControls();

    XboxController alternateController = new XboxController(1);
    TestControlsWithSwerve testControls = new TestControlsWithSwerve(mMainController, alternateController);
    mStateMachine = new StateMachine(testControls, mSwerveDrive, mIntake, mShooter, mExtendoArm, null);
    // mIntakingState = new IntakingState(mSwerveDrive, mIntake, mShooter, testControls);
    // mHoldingState = new HoldingState(mSwerveDrive, mIntake, mShooter, testControls);
    // mShootingState = new ShootingState(mSwerveDrive, mIntake, mShooter, testControls);
    
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
    //LogMotorAndSensorBasics();
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
    mSwerveDrive.EnableDiagnostics();
    mSwerveDrive.EnableDiagnostics(WheelLabel.NE);
    mSwerveDrive.EnableDiagnostics(WheelLabel.NW);
    mSwerveDrive.EnableDiagnostics(WheelLabel.SE);
    mSwerveDrive.EnableDiagnostics(WheelLabel.SW);
  }
  {
  /*
  private void AnglerSimplePID()
  {
    double triggers = mMainController.getLeftTriggerAxis() - mMainController.getRightTriggerAxis();
    if(triggers > 0.05 || triggers < -0.05)
    {
      mAngler.set(triggers);
    }
    else
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
        if(Math.abs(mAnglerEncoder.getPosition() - setpoint) < 0.5)
        {
          mAngler.stopMotor();  //break mode will generally hold us in place without drawing any current
        }
        else
        {
          mAngler.getPIDController().setReference(setpoint, ControlType.kPosition);
        }
      }
      else
      {
        mAngler.stopMotor();
      }
    }
  }

  private void AnglerBasicTest()
  {
    mAngler.set(mMainController.getLeftTriggerAxis() - mMainController.getRightTriggerAxis() );
  }

  private void ExtendoArmSimplePID()
  {
    double triggers = mMainController.getLeftTriggerAxis() - mMainController.getRightTriggerAxis();
    if(triggers > 0.05 || triggers < -0.05)
    {
      mExtendoArm.set(triggers);
    }
    else
    {
      double setpoint = -1000;//meaningless default
      boolean isButtonPressed = false;
      if(mMainController.getYButton())
      {
        setpoint = -1200;
        isButtonPressed = true;
      }
      else if(mMainController.getBButton())
      {
        setpoint = -900;
        isButtonPressed = true;
      }
      else if(mMainController.getAButton())
      {
        setpoint = -1800;
        isButtonPressed = true;
      }
      else if(mMainController.getXButton())
      {
        setpoint = -300;
        isButtonPressed = true;
      }

      if(isButtonPressed)
      {
        if(Math.abs(mAnglerEncoder.getPosition() - setpoint) < 10.0)
        {
          mExtendoArm.stopMotor();  //break mode will generally hold us in place without drawing any current
        }
        else
        {
          mExtendoArm.getPIDController().setReference(setpoint, ControlType.kPosition);
        }
      }
      else
      {
        mExtendoArm.stopMotor();
      }
    }
  }

  private void ExtendoArmBasicTest()
  {
     mExtendoArm.set(mMainController.getLeftTriggerAxis() - mMainController.getRightTriggerAxis());
  }

  private void SwerveTest()
  {
    mSwerveDrive.Run(mJoystickSwerveControls);
  }
  */
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
        mIntake.setToEjectingSpeed();
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

  public void logLimeLightInfo()
  {
    try {

      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      NetworkTableEntry jsonNTE = table.getEntry("json");
      String json_dump = jsonNTE.getString("null");

      double[] camerapose_targetspace = Functionality.getCameraPoseTargetSpace(json_dump);
      if (camerapose_targetspace != null)
      {
        SmartDashboard.putNumber("X displacement", camerapose_targetspace[0]);
        SmartDashboard.putNumber("Y displacement", camerapose_targetspace[1]);
        SmartDashboard.putNumber("Z displacement", camerapose_targetspace[2]);

        SmartDashboard.putNumber("Roll displacement", camerapose_targetspace[3]);
        SmartDashboard.putNumber("Pitch displacement", camerapose_targetspace[4]);
        SmartDashboard.putNumber("Yaw displacement", camerapose_targetspace[5]);
        SmartDashboard.putNumber("Distance from tag", Math.abs(camerapose_targetspace[0] - (-0.29)));
        SmartDashboard.putBoolean("Can See Tag", true);
      }
      else
      {
        SmartDashboard.putNumber("X displacement", -1);
        SmartDashboard.putNumber("Y displacement", -1);
        SmartDashboard.putNumber("Z displacement", -1);
        SmartDashboard.putNumber("Roll displacement", -1);
        SmartDashboard.putNumber("Pitch displacement", -1);
        SmartDashboard.putNumber("Yaw displacement", -1);
        SmartDashboard.putBoolean("Can See Tag", false);
        
      }
    }
    catch (Exception e)
    {
      SmartDashboard.putString("Error", e.getLocalizedMessage());
    }
  }

  public void zeroAnglerEncoderAndGyro()
  {
    // SmartDashboard.putBoolean("Limit Reached", mAnglerHardReverseLimitSwitch.isPressed());
    // SmartDashboard.putNumber("Angler Pos", mAnglerMotor.getEncoder().getPosition());
    
    if (!mAnglerMotorAndGyroZeroingHasOccurred)
    {
      mShooter.setAnglerSpeed(-0.05);
      
      if (mShooter.isAnglerHardReverseLimitSwitchPressed())
      {
        mSwerveDrive.SetGyroscopeCurrentAngle(0);
    
        mAnglerMotorAndGyroZeroingHasOccurred = true;
        mShooter.setAnglerSpeed(0);
        mShooter.zeroAnglerMotorInformation();
        Constants.setLowGoalRotation();
      }
    }
    else
    {
      SmartDashboard.putString("Status","Not Here");
    }
  }


  


  private double mPosition = -800;
  public void teleopPeriodic()
  {
    // SmartDashboard.putNumber("New Gyro Roll", mPigeon2.getRoll().getValueAsDouble());
    // SmartDashboard.putNumber("New Gyro Yaw", mPigeon2.getYaw().getValueAsDouble());
    // SmartDashboard.putNumber("New Gyro Pitch", mPigeon2.getPitch().getValueAsDouble());
    logLimeLightInfo();
    // SmartDashboard.putNumber("New Gyro Angle", mPigeon2.getRoll().getValueAsDouble());
    if (!mAnglerMotorAndGyroZeroingHasOccurred)
    {
      zeroAnglerEncoderAndGyro();
      return;
    }
    
    
    // mSwerveDrive.LogDiagnostics();
    if (mSwerveDrive.getLastRequestedChassisSpeeds() != null)
    {
      SmartDashboard.putNumber("Last Rq Chassis Speeds VY", mSwerveDrive.getLastRequestedChassisSpeeds().vyMetersPerSecond);
    }
    else
    {
      SmartDashboard.putNumber("Last Rq Chassis Speeds VY", -1);
    }
    mStateMachine.Run();
    
  
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
