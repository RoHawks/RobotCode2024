package robosystems;

import java.util.Arrays;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Functionality;
import frc.robot.LimelightInformation;
import frc.robot.LimelightManager;
import states.ShooterMode;
import states.ShootingState; 

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

    //private CANSparkFlex mAnglerMotor;
    private CANSparkMax mAnglerMotor;
    

    private SparkLimitSwitch mHardForwardLimitSwitch;
    private SparkLimitSwitch mHardReverseLimitSwitch;

    private boolean mHasCorrectSpeed;
    private double mShooterAtRightSpeedStartingTime;
    
    //private boolean shootingFinished;
    private SparkPIDController mPIDController;

    private final double mMoveBackwardsSpeed = -0.2;

    //private final double AUTOMATIC_SHOOTING_ANGLE = 50;

    //private boolean mHasSeenTheTag;

    private final VelocityVoltage mVoltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
    //private final com.ctre.phoenix6.controls.VelocityDutyCycle mVelocityDutyCycle = new VelocityDutyCycle(0, 0, true, 0, 0, false, false, false);

    private LimelightManager mLimelightManager;
    
    private Boolean mAnglerSoftLimitSwitchesEnabled = null;
    
    public void SetAnglerLimitSwitchesEnabledOrDisabled(boolean pEnabled)
    {

        if(mAnglerSoftLimitSwitchesEnabled == null || pEnabled != mAnglerSoftLimitSwitchesEnabled)
        {
            mAnglerMotor.enableSoftLimit(SoftLimitDirection.kForward, pEnabled);
            mAnglerMotor.enableSoftLimit(SoftLimitDirection.kReverse, pEnabled);
            mAnglerSoftLimitSwitchesEnabled = pEnabled;
        }
    }

    
    private TalonFX CreateShooterMotor(int pDeviceID, boolean pIsInverted, TalonFXConfiguration pShooterConfig,
    ClosedLoopRampsConfigs pClrc, CurrentLimitsConfigs pClc)
    {

        TalonFX returnValue =  new TalonFX(pDeviceID);  
        returnValue.setInverted(pIsInverted);
        returnValue.getConfigurator().apply(pShooterConfig);
        
        MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs();
        motorOutputConfig.NeutralMode = NeutralModeValue.Coast;
        returnValue.getConfigurator().apply(motorOutputConfig);
        returnValue.getConfigurator().apply(pClrc);
        returnValue.getConfigurator().apply(pClc);
        return returnValue;
    }

    public double GetAnglerEncoderReading()
    {
        return mAnglerMotor.getEncoder().getPosition();
    }

    private CANSparkMax CreateAnglerMotor(int pDeviceID, boolean pIsInverted)
    {
        CANSparkMax returnValue = new CANSparkMax(pDeviceID, MotorType.kBrushless);
        returnValue.setSmartCurrentLimit(39);      
        returnValue.setIdleMode(IdleMode.kCoast); // returnValue.setIdleMode(IdleMode.kBreak); 
        returnValue.setInverted(pIsInverted);
        returnValue.setSoftLimit(SoftLimitDirection.kForward, 190);
        returnValue.setSoftLimit(SoftLimitDirection.kReverse, 5);
        returnValue.enableSoftLimit(SoftLimitDirection.kForward, true);
        returnValue.enableSoftLimit(SoftLimitDirection.kReverse, true);

        returnValue.getPIDController().setP(    0.5); //was 0.12 at TVR
        returnValue.getPIDController().setI(0.00);
        returnValue.getPIDController().setD(0.00);
        returnValue.getPIDController().setFF(0.00);

        mHardForwardLimitSwitch = returnValue.getForwardLimitSwitch(Type.kNormallyOpen);
        mHardReverseLimitSwitch = returnValue.getReverseLimitSwitch(Type.kNormallyOpen);
        
        mHardForwardLimitSwitch.enableLimitSwitch(false);
        mHardReverseLimitSwitch.enableLimitSwitch(true);
        
        // mHardForwardLimitSwitch = returnValue.getForwardLimitSwitch(Type.kNormallyClosed);
        // mHardReverseLimitSwitch = returnValue.getReverseLimitSwiotch(Type.kNormallyClosed);
        // mHardForwardLimitSwitch.enableLimitSwitch(true);
        // mHardForwardLimitSwitch.enableLimitSwitch(true);

        returnValue.burnFlash();
        return returnValue;
    }

    public enum RollerEnumeration
    {
        TOP,
        BOTTOM
    }

    private ClosedLoopRampsConfigs GetClosedLoopRampsConfigs(RobotMode pRobotMode, RollerEnumeration pRoller)
    {
        ClosedLoopRampsConfigs returnValue = new ClosedLoopRampsConfigs();
        if(pRobotMode == RobotMode.AUTONOMOUS)
        {
            if(pRoller == RollerEnumeration.TOP)
            {
                returnValue.DutyCycleClosedLoopRampPeriod = 0.3;
                returnValue.VoltageClosedLoopRampPeriod = 0.3;
            }
            else //bottom
            {
                returnValue.DutyCycleClosedLoopRampPeriod = 0.5;
                returnValue.VoltageClosedLoopRampPeriod = 0.5;
            }
        }
        else //TELEOP
        {
            if(pRoller == RollerEnumeration.TOP)
            {
                returnValue.DutyCycleClosedLoopRampPeriod = 0.6;
                returnValue.VoltageClosedLoopRampPeriod = 0.6;
            }
            else //bottom
            {
                returnValue.DutyCycleClosedLoopRampPeriod = 0.5;
                returnValue.VoltageClosedLoopRampPeriod = 0.5;
            }
        }

        return returnValue;
    }
    
    private CurrentLimitsConfigs GetCurrentLimitsConfigs(RobotMode pRobotMode, RollerEnumeration pRoller)
    {
        CurrentLimitsConfigs returnValue = new CurrentLimitsConfigs();
        if(pRobotMode == RobotMode.AUTONOMOUS)
        {
            if(pRoller == RollerEnumeration.TOP)
            {
                returnValue.SupplyCurrentLimit = 70;//amps
                returnValue.SupplyCurrentLimitEnable = true;
            }
            else //bottom
            {
                returnValue.SupplyCurrentLimit = 38;//amps
                returnValue.SupplyCurrentLimitEnable = true;
            }
        }
        else //TELEOP
        {
            if(pRoller == RollerEnumeration.TOP)
            {
                returnValue.SupplyCurrentLimit = 50;//amps
                returnValue.SupplyCurrentLimitEnable = true;
            }
            else //bottom
            {
                returnValue.SupplyCurrentLimit = 38;//amps
                returnValue.SupplyCurrentLimitEnable = true;
            }
        }

        return returnValue;
    }
    
    public void ConfigureShooterMotorsForGameMode(RobotMode pMode)
    {
        mTopShooterRoller.getConfigurator().apply(GetClosedLoopRampsConfigs(pMode,RollerEnumeration.TOP));
        mBottomShooterRoller.getConfigurator().apply(GetClosedLoopRampsConfigs(pMode,RollerEnumeration.BOTTOM));
        mTopShooterRoller.getConfigurator().apply(GetCurrentLimitsConfigs(pMode,RollerEnumeration.TOP));
        mBottomShooterRoller.getConfigurator().apply(GetCurrentLimitsConfigs(pMode,RollerEnumeration.TOP));
    }

    public Shooter(LimelightManager pLimelightManager) // initialization method
    {
        mLimelightManager = pLimelightManager;

        mAnglerMotor =  CreateAnglerMotor(14, true); 
        
        TalonFXConfiguration topShooterConfig = new TalonFXConfiguration();            
        topShooterConfig.Slot0.kS = -0.065667;
        topShooterConfig.Slot0.kP = 0.13917;
        topShooterConfig.Slot0.kI = 0;
        topShooterConfig.Slot0.kD = 0.02;
        topShooterConfig.Slot0.kV = 0.16182;
        
        topShooterConfig.Slot0.kA = 0;

    
        mTopShooterRoller = CreateShooterMotor(4, true, topShooterConfig, GetClosedLoopRampsConfigs(RobotMode.TELEOPERATED, RollerEnumeration.TOP), GetCurrentLimitsConfigs(RobotMode.TELEOPERATED, RollerEnumeration.TOP));

        TalonFXConfiguration bottomShooterConfig = new TalonFXConfiguration();            
        bottomShooterConfig.Slot0.kS = -0.84808;
        bottomShooterConfig.Slot0.kP = 0.14289;
        bottomShooterConfig.Slot0.kI = 0;
        bottomShooterConfig.Slot0.kD = 0.02;
        bottomShooterConfig.Slot0.kV = 0.17996;
        bottomShooterConfig.Slot0.kA = 0;        

        mBottomShooterRoller = CreateShooterMotor(5, false, bottomShooterConfig, GetClosedLoopRampsConfigs(RobotMode.TELEOPERATED, RollerEnumeration.BOTTOM), GetCurrentLimitsConfigs(RobotMode.TELEOPERATED, RollerEnumeration.BOTTOM));

        mPIDController = mAnglerMotor.getPIDController();
        //shootingFinished = true;

        mHasCorrectSpeed = false;
        mShooterAtRightSpeedStartingTime = System.currentTimeMillis();

        //mHasSeenTheTag = false;
        resetOutputRange();

    }
    public boolean mAnglerHardReverseLimitSwitchisPressed()
    {
        return mHardReverseLimitSwitch.isPressed();
    }

    /* Not used 
    {
        mHasSeenTheTag = pHasSeenTheTag;
    }
    */

    public void setSpeed(double topSpeed, double bottomSpeed){ //set speed to integer -1.0 <= n <= 1.0
        if (Math.abs(topSpeed) > 0.01)
        {
            mTopShooterRoller.setControl(mVoltageVelocity.withVelocity(-topSpeed));
            //mTopShooterRoller.setControl(mVelocityDutyCycle.withVelocity(-topSpeed));
            
        }
        else
        {
            mTopShooterRoller.stopMotor();
        }

        if (Math.abs(bottomSpeed) > 0.01)
        {
            mBottomShooterRoller.setControl(mVoltageVelocity.withVelocity(bottomSpeed));
            //mBottomShooterRoller.setControl(mVelocityDutyCycle.withVelocity(bottomSpeed));
            
        }
        else
        {
            mBottomShooterRoller.stopMotor();
        }
        
        // shootingFinished = (speed == 0.0);
    }

    public double getTopSpeed()
    {   
        return mTopShooterRoller.getVelocity().getValueAsDouble();
    }

    public double getBottomSpeed()
    {   
        return mBottomShooterRoller.getVelocity().getValueAsDouble();
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

    public void setToSecondEjectingSpeed()
    {
        spinUpToHighGoalSpeed();
    }

    public void setToFirstEjectingSpeed()
    {
        setSpeed(-Constants.SHOOTER_HIGH_TOP_DEFAULT_SPEED, -Constants.SHOOTER_HIGH_BOTTOM_DEFAULT_SPEED);
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
                if (System.currentTimeMillis() - mShooterAtRightSpeedStartingTime > 200)
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

    public boolean testGetForwardAnglerLimitSwitchTriggered()
    {
        return mAnglerMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
    }

    public boolean testGetReverseAnglerLimitSwitchTriggered()
    {
        return mAnglerMotor.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
    }

    public void resetForHoldingState()
    {
        mHasCorrectSpeed = false;
    }



    public void logShooterInformation()
    {
        SmartDashboard.putNumber("Shooter: Top Shooter Velocity", mTopShooterRoller.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter: Top Output Voltage", mTopShooterRoller.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter: Top Shooter Current", mTopShooterRoller.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter: Bottom Shooter Velocity", mBottomShooterRoller.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter: Bottom Output Voltage", mBottomShooterRoller.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter: Bottom Shooter Current", mBottomShooterRoller.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter: Constant Top Shooter Speed", Constants.SHOOTER_HIGH_TOP_DEFAULT_SPEED);
        SmartDashboard.putNumber("Shooter: Constant Bottom Shooter Speed", Constants.SHOOTER_HIGH_BOTTOM_DEFAULT_SPEED);
        SmartDashboard.putNumber("Shooter: Time Elapsed Since Shooter Reached Right Speed", System.currentTimeMillis() - mShooterAtRightSpeedStartingTime);
        SmartDashboard.putNumber("Shooter: Angler Output Percentage", mAnglerMotor.getAppliedOutput() + Math.random()/1000.0);
        SmartDashboard.putNumber("Shooter: Angler Output Current", mAnglerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter: Angler Current Encoder Reading", GetAnglerEncoderReading() );
        SmartDashboard.putNumber("Shooter: Angler Target Angle", mLastTargetAngle);
        

    }   


    public void setAngleBasedOnShooterMode(ShooterMode pShooterMode)
    {
        setAngleBasedOnShooterMode(pShooterMode, null);
    }

    public void setAngleBasedOnShooterMode(ShooterMode pShooterMode, ChassisSpeeds pChassisSpeeds)
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
            automaticAngle(pChassisSpeeds);
        }
        else if (pShooterMode == ShooterMode.AutoAim)
        {
            automaticAngleWithRotationImplementation(pChassisSpeeds);
        }
        
    }

    public void ResetNonCameraAutoAimAngle()
    {
        mLastCalculatedAutoAngleFromCamera = Constants.DEFAULT_AUTO_AIM_ANGLE;
    }


    //In case we very briefly lose view of the AprilTag with camera, this variable will keep track
    //of the last time we saw it, and we'll just reuse that.
    private double mLastCalculatedAutoAngleFromCamera = Constants.DEFAULT_AUTO_AIM_ANGLE;
    
    public double calculateAutoAngle(ChassisSpeeds pChassisSpeeds)
    {
        double[] cameraPoseTargetSpace = mLimelightManager.getCameraPoseTargetSpace();
        if (cameraPoseTargetSpace == null)
        {
            //return Constants.DEFAULT_AUTO_AIM_ANGLE;
            return mLastCalculatedAutoAngleFromCamera;
        }
        double yDisplacement = 1.12;

        double zDisplacement = -1 * cameraPoseTargetSpace[2];

        //SmartDashboard.putNumber("Lime Calcs: zDisplacement", zDisplacement);

        double adjustedYDisplacement = yDisplacement * (83.0/57.0);

        //SmartDashboard.putNumber("Lime Calcs: adjustedYDisplacement ", adjustedYDisplacement );

        double adjustedZDisplacement = zDisplacement + (0.36);

        //SmartDashboard.putNumber("Lime Calcs: adjustedZDisplacement ", adjustedZDisplacement );
        
        double calculatedAngle = Math.toDegrees(Math.atan2(adjustedYDisplacement, adjustedZDisplacement));
        
        //SmartDashboard.putNumber("Lime Calcs: angleToTheShooter ", calculatedAngle);
        
        calculatedAngle = calculatedAngle - Constants.INITIAL_SHOOTER_ANGLE_TO_ACCOUNT_FOR;


        calculatedAngle = calculatedAngle * (1.0 / Constants.ANGLER_ROTATIONS_TO_ANGLES);

        
        double maxSpeedMetersPerSecond = 5.0;
        double adjustmentFromZVelocity;
        if (pChassisSpeeds != null)
        {
            //SmartDashboard.putNumber("pChassisSpeeds.vxMetersPerSecond", pChassisSpeeds.vxMetersPerSecond);
            double percentageOfMaximumZSpeed = pChassisSpeeds.vxMetersPerSecond / maxSpeedMetersPerSecond;
            adjustmentFromZVelocity = percentageOfMaximumZSpeed * Constants.Z_VELOCITY_COMPENSATION;
        }
        else
        {
            adjustmentFromZVelocity = 0;
        }
        //SmartDashboard.putNumber("Lime Calcs: adjustmentFromZVelocity", adjustmentFromZVelocity);

        calculatedAngle += adjustmentFromZVelocity;

        double adamOffset = -15;
        calculatedAngle += adamOffset;
        //SmartDashboard.putNumber("Angle to shoot at", calculatedAngle);

        mLastCalculatedAutoAngleFromCamera = calculatedAngle;
        return calculatedAngle;
    }
    
    public double calculateAutoAngle_MegaTagBotPose(ChassisSpeeds pChassisSpeeds)
    {
        double[] botPose = mLimelightManager.getBotPoseFromCameraBasedOnChassisSpeeds(pChassisSpeeds);
        if (!LimelightInformation.isValidBotPoseResults(botPose))
        {
            //return Constants.DEFAULT_AUTO_AIM_ANGLE;
            return mLastCalculatedAutoAngleFromCamera;
        }
        double yDisplacement = 1.12;

        double zDisplacement = LimelightInformation.BotPose_GetDistanceFromAllianceStationWall(botPose);

        //SmartDashboard.putNumber("Lime Calcs: zDisplacement", zDisplacement);

        double adjustedYDisplacement = yDisplacement * (83.0/57.0);

        //SmartDashboard.putNumber("Lime Calcs: adjustedYDisplacement ", adjustedYDisplacement );

        double adjustedZDisplacement = zDisplacement + (0.36);

        //SmartDashboard.putNumber("Lime Calcs: adjustedZDisplacement ", adjustedZDisplacement );
        
        double calculatedAngle = Math.toDegrees(Math.atan2(adjustedYDisplacement, adjustedZDisplacement));
        
        //SmartDashboard.putNumber("Lime Calcs: angleToTheShooter ", calculatedAngle);
        
        calculatedAngle = calculatedAngle - Constants.INITIAL_SHOOTER_ANGLE_TO_ACCOUNT_FOR;


        calculatedAngle = calculatedAngle * (1.0 / Constants.ANGLER_ROTATIONS_TO_ANGLES);

        
        double maxSpeedMetersPerSecond = 5.0;
        double adjustmentFromZVelocity;
        if (pChassisSpeeds != null)
        {
            //SmartDashboard.putNumber("pChassisSpeeds.vxMetersPerSecond", pChassisSpeeds.vxMetersPerSecond);
            double percentageOfMaximumZSpeed = pChassisSpeeds.vxMetersPerSecond / maxSpeedMetersPerSecond;
            adjustmentFromZVelocity = percentageOfMaximumZSpeed * Constants.Z_VELOCITY_COMPENSATION;
        }
        else
        {
            adjustmentFromZVelocity = 0;
        }
        //SmartDashboard.putNumber("Lime Calcs: adjustmentFromZVelocity", adjustmentFromZVelocity);

        calculatedAngle += adjustmentFromZVelocity;

        double adamOffset = -20;
        calculatedAngle += adamOffset;
        //SmartDashboard.putNumber("Angle to shoot at", calculatedAngle);

        mLastCalculatedAutoAngleFromCamera = calculatedAngle;
        return calculatedAngle;
    }



    public double calculateAutoAngleWithRotations_MegaTagBotPose(ChassisSpeeds pChassisSpeeds)
    {
        return calculateAutoAngleWithRotations_MegaTagBotPose(mLimelightManager.getBotPoseFromCameraBasedOnChassisSpeeds(pChassisSpeeds), pChassisSpeeds);
    }

    public double calculateAutoAngleWithRotations_MegaTagBotPose_WithCameraPreference(int pCamera)
    {
        return calculateAutoAngleWithRotations_MegaTagBotPose(mLimelightManager.getBotPoseByPriorityCamera(pCamera), null);
    }

    public double calculateAutoAngleWithRotations_MegaTagBotPose(double[] pBotPose, ChassisSpeeds pChassisSpeeds)
    {        
        if (!LimelightInformation.isValidBotPoseResults(pBotPose))
        {
            return mLastCalculatedAutoAngleFromCamera;
        }
        double yDisplacement = 1.12;

        double zDisplacement = LimelightInformation.BotPose_GetDistanceFromAllianceStationWall(pBotPose);
        double xDisplacement = LimelightInformation.BotPose_GetHorizontalDistanceFromAprilTag(pBotPose);

        SmartDashboard.putNumber("AutoLog: xDisplacement", xDisplacement);

        double adjustedYDisplacement = yDisplacement * (83.0/57.0);

        double adjustedZDisplacement = zDisplacement + (0.36);

        SmartDashboard.putNumber("AutoLog: xDisplacement", adjustedZDisplacement);

        double hypotenuse = Math.sqrt(Math.pow(adjustedZDisplacement,2) + Math.pow(xDisplacement,2));

        double calculatedAngle = Math.toDegrees(Math.atan2(adjustedYDisplacement, hypotenuse));
    
        
        calculatedAngle = calculatedAngle - Constants.INITIAL_SHOOTER_ANGLE_TO_ACCOUNT_FOR;


        calculatedAngle = calculatedAngle * (1.0 / Constants.ANGLER_ROTATIONS_TO_ANGLES);

        
        double maxSpeedMetersPerSecond = 5.0;
        double adjustmentFromZVelocity;
        if (pChassisSpeeds != null)
        {
            double percentageOfMaximumZSpeed = pChassisSpeeds.vxMetersPerSecond / maxSpeedMetersPerSecond;
            adjustmentFromZVelocity = percentageOfMaximumZSpeed * Constants.Z_VELOCITY_COMPENSATION;
        }
        else
        {
            adjustmentFromZVelocity = 0;
        }

        calculatedAngle += adjustmentFromZVelocity;

        mLastCalculatedAutoAngleFromCamera = calculatedAngle;

        double adamOffset = -7;
        calculatedAngle += adamOffset;

        SmartDashboard.putNumber("AutoLog: calculatedAngle", calculatedAngle);
        return calculatedAngle;
    }


    public void automaticAngle(ChassisSpeeds pChassisSpeeds) 
    {
        double angle;
        if(ShootingState.USE_MEGA_TAG_BOT_POSE)
        {
            angle = calculateAutoAngle_MegaTagBotPose(pChassisSpeeds);
        }
        else
        {
            angle = calculateAutoAngle(pChassisSpeeds);
        }
        
        
        setAngle(angle);

    }

    public void automaticAngleWithRotationImplementation(ChassisSpeeds pChassisSpeeds) 
    {
        double angle;
        if(ShootingState.USE_MEGA_TAG_BOT_POSE)
        {
            if(ShootingState.TECH_VALLEY_AUTO_SHOOTING_VERSION)
            {
                angle = calculateAutoAngleWithRotations_MegaTagBotPose(pChassisSpeeds);
            }
            else
            {
                angle = calculateAutoAngleWithRotations_MegaTagBotPose_WithCameraPreference(LimelightManager.EAST_CAMERA);
            }
        }
        else
        {
            SmartDashboard.putString("THERE IS AN ERROR IN THE CODE", "PLEASE NOTICE ME");
            angle = calculateAutoAngle(pChassisSpeeds);
        }
        
        
        setAngle(angle);

    }

    public boolean getIfAutomaticAnglerInRange(ChassisSpeeds pChassisSpeeds)
    {
         double angle;
        if(ShootingState.USE_MEGA_TAG_BOT_POSE)
        {
            angle = calculateAutoAngle_MegaTagBotPose(pChassisSpeeds);
        }
        else
        {
            angle = calculateAutoAngle(pChassisSpeeds);
        }
        double calculatedError = angle - mAnglerMotor.getEncoder().getPosition();
        



        if (Math.abs(calculatedError) < Constants.ANGLER_ACCEPTABLE_ERROR)
        {
            return true;
        }
        else
        {
            return false;
        }
    }


    private double mLastTargetAngle;
    public void setAngle(double pDesiredPosition) //needs to be implemented
    {
        //SmartDashboard.putNumber("EXCITING: Last Requested Angle To Shoot At", pDesiredPosition);
        mLastTargetAngle = pDesiredPosition;
        mAnglerMotor.getPIDController().setReference(pDesiredPosition, ControlType.kPosition);

    }

    public void enterTrapShootingStart()
    {
        mPIDController.setOutputRange(-0.3, 0.3);
    }

    public void goToTrapShootAngle()
    {

        //SmartDashboard.putNumber("Encoder says ", mAnglerMotor.getEncoder().getPosition());
        mAnglerMotor.getPIDController().setReference(Constants.CLIMBING_ANGLE, ControlType.kPosition);
    
    }

    
    public void resetOutputRange()
    {
        mPIDController.setOutputRange(-1000, 1000);
    }

    public boolean checkIfAnglerIsCloseEnough(double pTarget)
    {
        double calculatedError = pTarget - mAnglerMotor.getEncoder().getPosition();        
        if (Math.abs(calculatedError) < Constants.ANGLER_ACCEPTABLE_ERROR)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    

    public boolean checkIfAnglerIsCloseEnough()
    {
        double calculatedError = mLastTargetAngle - mAnglerMotor.getEncoder().getPosition();        
        if (Math.abs(calculatedError) < Constants.ANGLER_ACCEPTABLE_ERROR)
        {
            return true;
        }
        else
        {
            return false;
        }
    }


    public void setAngleToIntakingAngle() //needs to be implemented
    {
        setAngle(Constants.INTAKING_ANGLE);
    }

    public void stopAnglerMotor()
    {
        mAnglerMotor.stopMotor();
    }

    public void logShooterAngle()
    {
        SmartDashboard.putNumber("Shooter Angle", mAnglerMotor.getEncoder().getPosition());

    }

    public java.util.List<TalonFX> getSpeakers()
    {
        return Arrays.asList(mTopShooterRoller, mBottomShooterRoller);
    }

    public void TestOnly_SetAnglerSpeed(double pSpeed)
    {
        mAnglerMotor.set(pSpeed);
    }
 
    public void TestOnly_SetTopShooterSpeed(double pSpeed)
    {
        mTopShooterRoller.set(pSpeed);
    }

    public void TestOnly_SetBottomShooterSpeed(double pSpeed)
    {
        mBottomShooterRoller.set(pSpeed);
    }
    
}