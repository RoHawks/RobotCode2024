package robosystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
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
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Functionality;
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

    private SparkLimitSwitch mHardForwardLimitSwitch;
    private SparkLimitSwitch mHardReverseLimitSwitch;

    private boolean mHasCorrectSpeed;
    private double mShooterAtRightSpeedStartingTime;
    
    private boolean shootingFinished;
    private SparkPIDController mPIDController;

    private final double mMoveBackwardsSpeed = -0.2;

    private final double AUTOMATIC_SHOOTING_ANGLE = 50;

    private final VelocityVoltage mVoltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  

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
        
        ClosedLoopRampsConfigs clrc = new ClosedLoopRampsConfigs();
        clrc.DutyCycleClosedLoopRampPeriod = 0.5;
        clrc.VoltageClosedLoopRampPeriod = 0.5;

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
        returnValue.enableSoftLimit(SoftLimitDirection.kForward, false);
        returnValue.enableSoftLimit(SoftLimitDirection.kReverse, false);

        returnValue.getPIDController().setP(0.12);
        returnValue.getPIDController().setI(0.00);
        returnValue.getPIDController().setD(0.00);
        returnValue.getPIDController().setFF(0.00);

        mHardForwardLimitSwitch = returnValue.getForwardLimitSwitch(Type.kNormallyOpen);
        mHardReverseLimitSwitch = returnValue.getReverseLimitSwitch(Type.kNormallyOpen);
        mHardForwardLimitSwitch.enableLimitSwitch(true);
        mHardReverseLimitSwitch.enableLimitSwitch(true);
        
        // mHardForwardLimitSwitch = returnValue.getForwardLimitSwitch(Type.kNormallyClosed);
        // mHardReverseLimitSwitch = returnValue.getReverseLimitSwiotch(Type.kNormallyClosed);
        // mHardForwardLimitSwitch.enableLimitSwitch(true);
        // mHardForwardLimitSwitch.enableLimitSwitch(true);

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

        mHasCorrectSpeed = false;
        mShooterAtRightSpeedStartingTime = System.currentTimeMillis();
        
    }

    

    public void setSpeed(double topSpeed, double bottomSpeed){ //set speed to integer -1.0 <= n <= 1.0
        if (Math.abs(topSpeed) > 0.01)
        {
            mTopShooterRoller.setControl(mVoltageVelocity.withVelocity(-topSpeed));
        }
        else
        {
            mTopShooterRoller.stopMotor();
        }

        if (Math.abs(bottomSpeed) > 0.01)
        {
            mBottomShooterRoller.setControl(mVoltageVelocity.withVelocity(bottomSpeed));
        }
        else
        {
            mBottomShooterRoller.stopMotor();
        }
        
        // shootingFinished = (speed == 0.0);
    }

    public void setAnglerSpeed(double pSpeed)
    {
        mAnglerMotor.set(pSpeed);
    }
    
    public void zeroAnglerMotorInformation()
    {
        mAnglerMotor.getEncoder().setPosition(0);
        mAnglerMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        mAnglerMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        mAnglerMotor.burnFlash();
    }

    public boolean isAnglerHardReverseLimitSwitchPressed()
    {
        return mHardReverseLimitSwitch.isPressed();
    }

    public void spinUpToHighGoalSpeed()
    {
        setSpeed(Constants.SHOOTER_HIGH_TOP_DEFAULT_SPEED, Constants.SHOOTER_HIGH_BOTTOM_DEFAULT_SPEED);
    }
    public void spinUpToLowGoalSpeed()
    {
        setSpeed(Constants.SHOOTER_LOW_TOP_DEFAULT_SPEED, Constants.SHOOTER_LOW_BOTTOM_DEFAULT_SPEED);
    }

    public void backingUpNoteToPreventFallingOut(){ // to prevent note escape
        mTopShooterRoller.set(-mMoveBackwardsSpeed);
        mBottomShooterRoller.set(-mMoveBackwardsSpeed);
    }
    
    private double getTopShooterVelocity()
    {
        return -mTopShooterRoller.getVelocity().getValueAsDouble();
    }



    public boolean checkIfPersistentlyHasCorrectSpeed(ShooterMode pShooterMode)
    {
        double topSpeed;
        double bottomSpeed;
        if (pShooterMode == ShooterMode.LowGoal)
        {
            topSpeed = Constants.SHOOTER_LOW_TOP_DEFAULT_SPEED;
            bottomSpeed = Constants.SHOOTER_LOW_BOTTOM_DEFAULT_SPEED;
        } 
        else
        {
            topSpeed = Constants.SHOOTER_HIGH_TOP_DEFAULT_SPEED;
            bottomSpeed = Constants.SHOOTER_HIGH_BOTTOM_DEFAULT_SPEED;
        }
         // Should be a global variable at the beginning
        double lowerPercent = 0.8;
        double higherPercent = 1.2;

        boolean bothShootersAreAboveLowerBound = getTopShooterVelocity()
            > topSpeed * lowerPercent && 
            mBottomShooterRoller.getVelocity().getValueAsDouble() 
            > bottomSpeed * lowerPercent;

        boolean bothShootersAreAboveHigherBound = getTopShooterVelocity()
            < topSpeed * higherPercent && 
            mBottomShooterRoller.getVelocity().getValueAsDouble() 
            < bottomSpeed * higherPercent;

        if (bothShootersAreAboveLowerBound && 
            bothShootersAreAboveHigherBound)
        {
            SmartDashboard.putNumber("Time Elapsed Since Reached Right Shooting Speed", System.currentTimeMillis() - mShooterAtRightSpeedStartingTime);
            if (!mHasCorrectSpeed)
            {
                mShooterAtRightSpeedStartingTime = System.currentTimeMillis();
                mHasCorrectSpeed = true;
                return false;
            }
            else
            {
                mHasCorrectSpeed = true;
                if (System.currentTimeMillis() - mShooterAtRightSpeedStartingTime > 1000)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

        }
        else
        {
            SmartDashboard.putNumber("Time Elapsed Since Reached Right Shooting Speed", -1);
            mHasCorrectSpeed = false;
            return false;
        }
    }

    public void resetForHoldingState()
    {
        mHasCorrectSpeed = false;
    }



    public void logShooterInformation()
    {
        SmartDashboard.putNumber("Shooter: Top Shooter Velocity", mTopShooterRoller.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter: Bottom Shooter Velocity", mBottomShooterRoller.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter: Constant Top Shooter Speed", Constants.SHOOTER_HIGH_TOP_DEFAULT_SPEED);
        SmartDashboard.putNumber("Shooter: Constant Bottom Shooter Speed", Constants.SHOOTER_HIGH_BOTTOM_DEFAULT_SPEED);
        SmartDashboard.putNumber("Shooter: Time Elapsed Since Shooter Reached Right Speed", System.currentTimeMillis() - mShooterAtRightSpeedStartingTime);
    }   

    public void setAngleBasedOnShooterMode(ShooterMode pShooterMode)
    {
        if (pShooterMode == ShooterMode.LowGoal)
        {
            setAngle(Constants.LOW_GOAL_ANGLE);
        }
        else if (pShooterMode == ShooterMode.HighGoalManual)
        {
            setAngle(Constants.HIGH_GOAL_ANGLE);
        }
        else if (pShooterMode == ShooterMode.HighGoalDriveBy)
        {
            automaticAngle();
        }
        else if (pShooterMode == ShooterMode.AutoAim)
        {
            automaticAngle();
        }
        
    }



    /**
     * Returns a boolean of if it's within an acceptable degree of what angle it wants to be in
     * @return
     */
    public void automaticAngle() 
    {
        double[] limelightInfo = Functionality.getLimeLightInfo();
        if (limelightInfo == null)
        {
            setAngle(Constants.DEFAULT_AUTO_AIM_ANGLE);
          
        } 
        else
        {
            double yDisplacement = limelightInfo[1];

            double zDisplacement = -1 * limelightInfo[2];

            // SmartDashboard.putNumber("zDisplacement", zDisplacement);

            double adjustedYDisplacement = yDisplacement * (83.0/57.0);

            // SmartDashboard.putNumber("adjustedYDisplacement ", adjustedYDisplacement );

            double adjustedZDisplacement = zDisplacement + (0.36);

            // SmartDashboard.putNumber("adjustedZDisplacement ", adjustedZDisplacement );
            
            double angleToTheShooter = Math.toDegrees(Math.atan2(adjustedYDisplacement, adjustedZDisplacement));
            
            // SmartDashboard.putNumber("angleToTheShooter ", angleToTheShooter );
            
            angleToTheShooter = angleToTheShooter - 13.5;


            angleToTheShooter = angleToTheShooter * (1.0 / Constants.ANGLER_ROTATIONS_TO_ANGLES);

            // angleToTheShooter = ((angleToTheShooter - 50.0) * 0.625) + 50;

            SmartDashboard.putNumber("Angle to shoot at", angleToTheShooter);
            setAngle(angleToTheShooter);
            
        }

    }

    public boolean getIfAutomaticAnglerInRange()
    {
        double[] limelightInfo = Functionality.getLimeLightInfo();
        if (limelightInfo == null)
        {
            return false;
        }

        double yDisplacement = limelightInfo[1];
        double zDisplacement = -1 * limelightInfo[2];
        double adjustedYDisplacement = yDisplacement * (83.0/57.0);
        double adjustedZDisplacement = zDisplacement + (0.36);
        double angleToTheShooter = Math.toDegrees(Math.atan2(adjustedYDisplacement, adjustedZDisplacement));
        angleToTheShooter = angleToTheShooter - 13.5;
        angleToTheShooter = angleToTheShooter * (1.0 / Constants.ANGLER_ROTATIONS_TO_ANGLES);
        if (Math.abs(angleToTheShooter - mAnglerMotor.getEncoder().getPosition()) < Constants.ANGLER_ACCEPTABLE_ERROR)
        {
            return true;
        }
        else
        {
            return false;
        }
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