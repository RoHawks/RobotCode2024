package robosystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import states.ShooterMode; 

/*
 * package robosystems;

import states.ShooterMode;

public class Shooter {

    private TalonFX mLeftMotor;
    private TalonFX mRightMotor;

    private CANSparkFlex mAnglerMotor;
    private PositionVoltage mPositionVoltage;

    
    public Shooter(){}

    public void setAngleToIntakingAngle() {};
    public void setAngle() {};
    public void setSpeed() {};
    public boolean doneShooting() { return false; }

    public void automaticAngle() {}
    public boolean seesShootingAprilTag() { return false;}

    public void setAngleBasedOnShooterMode(ShooterMode pShooterMode) {}
}

 */

public class Shooter{
    private TalonFX mTopShooterRoller;
    private TalonFX mBottomShooterRoller;

    private CANSparkFlex mAnglerMotor;
    private PositionVoltage mPositionVoltage;

    private boolean shootingFinished;
    private SparkPIDController mPIDController;

    private final double mMoveBackwardsSpeed = -0.3;

    private final double AUTOMATIC_SHOOTING_ANGLE = 50;

    private final VelocityVoltage mVoltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
  

    private TalonFX CreateShooterMotor(int pDeviceID, boolean pIsInverted)
    {

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();    
        shooterConfig.Slot0.kP = 0.2;
        shooterConfig.Slot0.kI = 0;
        shooterConfig.Slot0.kD = 0;
        shooterConfig.Slot0.kV = 0.137;

       

        TalonFX returnValue =  new TalonFX(pDeviceID);  
        returnValue.setInverted(pIsInverted);
        returnValue.getConfigurator().apply(shooterConfig);
        
        MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs();
        motorOutputConfig.NeutralMode = NeutralModeValue.Coast;
        returnValue.getConfigurator().apply(motorOutputConfig);

        return returnValue;
    }


    private CANSparkFlex CreateAnglerMotor(int pDeviceID, boolean pIsInverted)
    {
        CANSparkFlex returnValue = new CANSparkFlex(pDeviceID, MotorType.kBrushless);
        returnValue.setSmartCurrentLimit(39);      
        returnValue.setIdleMode(IdleMode.kBrake);
        returnValue.setInverted(pIsInverted);
        returnValue.setSoftLimit(SoftLimitDirection.kForward, 190);
        returnValue.setSoftLimit(SoftLimitDirection.kReverse, 10);
        returnValue.enableSoftLimit(SoftLimitDirection.kForward, true);
        returnValue.enableSoftLimit(SoftLimitDirection.kReverse, true);

        returnValue.getPIDController().setP(0.08);
        returnValue.getPIDController().setI(0.00);
        returnValue.getPIDController().setD(0.00);
        returnValue.getPIDController().setFF(0.00);


        returnValue.burnFlash();
        return returnValue;
    }

    public Shooter() // initialization method
    {
        mAnglerMotor =  CreateAnglerMotor(14, true); 
        mTopShooterRoller = CreateShooterMotor(4, true);
        mBottomShooterRoller = CreateShooterMotor(5, false);

        mPIDController = mAnglerMotor.getPIDController();
       

        shootingFinished = true;
        
    }

    

    public void setSpeed(double topSpeed, double bottomSpeed){ //set speed to integer -1.0 <= n <= 1.0
        mTopShooterRoller.setControl(mVoltageVelocity.withVelocity(-topSpeed));
        mBottomShooterRoller.setControl(mVoltageVelocity.withVelocity(bottomSpeed));

        // shootingFinished = (speed == 0.0);
    }
    
    public void shootAtDefaultSpeed()
    {
        setSpeed(Constants.SHOOTER_TOP_DEFAULT_SPEED, Constants.SHOOTER_BOTTOM_DEFAULT_SPEED);
    }

    public void backingUpNoteToPreventFallingOut(){ // to prevent note escape
        mTopShooterRoller.set(mMoveBackwardsSpeed);
        mBottomShooterRoller.set(mMoveBackwardsSpeed);
    }
    
    public void setAngleBasedOnShooterMode(ShooterMode pShooterMode)
    {
        if (pShooterMode == ShooterMode.LowGoal)
        {
            setAngle(Constants.LOW_GOAL_ANGLE);
        }
        else if (pShooterMode == ShooterMode.HighGoalManual || 
                 pShooterMode == ShooterMode.HighGoalDriveBy)
        {
            setAngle(Constants.HIGH_GOAL_ANGLE);
        }
        else if (pShooterMode == ShooterMode.AutoAim)
        {
            automaticAngle();
        }
        
    }

    public void automaticAngle() 
    {
        setAngle(AUTOMATIC_SHOOTING_ANGLE);

    }
    public void setAngle(double pDesiredPosition) //needs to be implemented
    {
        mAnglerMotor.getPIDController().setReference(pDesiredPosition, ControlType.kPosition);

    }

    public void setAngleToIntakingAngle() //needs to be implemented
    {
        setAngle(Constants.INTAKING_ANGLE);
    }

    public void stopAnglerMotor()
    {
        mAnglerMotor.stopMotor();
    }

    public void logIntakeValues()
    {
        SmartDashboard.putNumber("Shooter Angle", mAnglerMotor.getEncoder().getPosition());

    }
    
}