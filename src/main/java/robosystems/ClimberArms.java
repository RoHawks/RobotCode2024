package robosystems;


import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;



public class ClimberArms 
{
  private TalonFX mLeftDrumMotor;
  private TalonFX mRightDrumMotor;


  private double RIGHT_EXTEND_TARGET_POSITION = Constants.RIGHT_EXTEND_TARGET_POSITION; 
  private double LEFT_EXTEND_TARGET_POSITION = Constants.LEFT_EXTEND_TARGET_POSITION; 
  private double RETRACT_TARGET_POSITION = Constants.RETRACT_TARGET_POSITION;
  
  private final PositionVoltage mPositionVoltage = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  
  private double mLeftLastTargetPosition = 0;
  private double mRightLastTargetPosition = 0;

  // Constructor
  public ClimberArms() {
      mLeftDrumMotor = new TalonFX(11);
      mRightDrumMotor = new TalonFX(12);

      
      configureTalon(mLeftDrumMotor);
      configureTalon(mRightDrumMotor);

      SoftwareLimitSwitchConfigs softLimitConfigsLeft = new SoftwareLimitSwitchConfigs();
      softLimitConfigsLeft.ForwardSoftLimitEnable = true;
      softLimitConfigsLeft.ForwardSoftLimitThreshold = 312;      
      mLeftDrumMotor.getConfigurator().apply(softLimitConfigsLeft);

      SoftwareLimitSwitchConfigs softLimitConfigsRight = new SoftwareLimitSwitchConfigs();
      softLimitConfigsRight.ReverseSoftLimitEnable = true;
      softLimitConfigsRight.ReverseSoftLimitThreshold = -328;
      mRightDrumMotor.getConfigurator().apply(softLimitConfigsRight);

  }

  // Configure Drum Motors
  public static void configureTalon(TalonFX pDrumMotor) {
      ClosedLoopRampsConfigs clrc = new ClosedLoopRampsConfigs();
      clrc.DutyCycleClosedLoopRampPeriod = 0.1;
      clrc.VoltageClosedLoopRampPeriod = 0.1;

      

      CurrentLimitsConfigs clc = new CurrentLimitsConfigs();
      clc.SupplyCurrentLimit = 39;
      clc.SupplyCurrentThreshold = 0.1;
      clc.SupplyCurrentLimitEnable = true;

      TalonFXConfiguration motorConfig = new TalonFXConfiguration();            
      motorConfig.Slot0.kP = 0.8;
      motorConfig.Slot0.kI = 0;
      motorConfig.Slot0.kD = 0;
      motorConfig.Slot0.kV = 0;
      motorConfig.Slot0.kA = 0;


      MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs();
      motorOutputConfig.NeutralMode = NeutralModeValue.Brake;
  
      pDrumMotor.getConfigurator().apply(clrc);
      pDrumMotor.getConfigurator().apply(clc);
      pDrumMotor.getConfigurator().apply(motorConfig);
      pDrumMotor.getConfigurator().apply(motorOutputConfig);


 

      

      
      
  }

  private boolean isArmCloseEnough(TalonFX pArmMotor, double pTargetPosition){
    return Math.abs(pArmMotor.getPosition().getValueAsDouble()-pTargetPosition) <= Constants.ARM_ACCEPTABLE_ERROR;
  }

  // Extends arms until both reach EXTEND_TARGET_POSITION

  private final double rightModifier = -1.0;
  public void goToPosition(double pLeftPosition, double pRightPosition)
  {
    mLeftDrumMotor.setControl(mPositionVoltage.withPosition(pLeftPosition));
    mRightDrumMotor.setControl(mPositionVoltage.withPosition(pRightPosition * rightModifier));
   
   
    mLeftLastTargetPosition = pLeftPosition;
    mRightLastTargetPosition = pRightPosition;
    
  }

  /*
  public void setSpeed(double pSpeed)
  {
    mLeftDrumMotor.set(pSpeed);
    mRightDrumMotor.set(pSpeed * rightModifier);
  }


  public void setLeftSpeed(double pSpeed)
  {
    mLeftDrumMotor.set(pSpeed);
    // mRightDrumMotor.set(pSpeed * rightModifier);
  }
  public void setRightSpeed(double pSpeed)
  {
    // mLeftDrumMotor.set(pSpeed);
    mRightDrumMotor.set(pSpeed * rightModifier);
  }

  public double getLeftPosition()
  {
    return mLeftDrumMotor.getPosition().getValueAsDouble();
  }

  public double getRightPosition()
  {
    return mRightDrumMotor.getPosition().getValueAsDouble();
  }
  */

  public boolean extend() {

      goToPosition(LEFT_EXTEND_TARGET_POSITION, RIGHT_EXTEND_TARGET_POSITION);

      if(isArmCloseEnough(mLeftDrumMotor, LEFT_EXTEND_TARGET_POSITION) && isArmCloseEnough(mRightDrumMotor, RIGHT_EXTEND_TARGET_POSITION * rightModifier)){
        return true;
      }else{
        return false;
      }
  }

  // Retracts arms until both reach RETRACT_TARGET_POSITION
  public boolean retract() {
      goToPosition(RETRACT_TARGET_POSITION,RETRACT_TARGET_POSITION);

      if(isArmCloseEnough(mLeftDrumMotor, RETRACT_TARGET_POSITION) && isArmCloseEnough(mRightDrumMotor, RETRACT_TARGET_POSITION * rightModifier)){
        return true;
      }else{
        return false;
      }
  }


  public void logArms()
  {
    logArm("LeftArm", mLeftDrumMotor);
    logArm("RightArm", mRightDrumMotor);
    SmartDashboard.putNumber("LeftArm-TargetPosition", mLeftLastTargetPosition);
    SmartDashboard.putNumber("RightArm-TargetPosition", mRightLastTargetPosition);
  }

  private void logArm(String pPrefix, TalonFX pMotor)
  {
    SmartDashboard.putNumber(pPrefix + "-Voltage", pMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(pPrefix + "-Current", pMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(pPrefix + "-Position", pMotor.getPosition().getValueAsDouble());


  }

  public void TEST_ONLY_SetLeftArmSpeed(double pSpeed)
  {
    mLeftDrumMotor.set(pSpeed);
  }

  public void TEST_ONLY_SetRightArmSpeed(double pSpeed)
  {
    mRightDrumMotor.set(pSpeed);
  }


}