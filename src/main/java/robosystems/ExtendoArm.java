package robosystems;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ExtendoArm {
    private CANSparkMax mExtendoMotor;
    private RelativeEncoder mExtendoEncoder;
    private SparkLimitSwitch mTrapIntakeLimitSwitch;
    
    private double LOW_GOAL_TARGET = 62.404 * 1.07; //Replace with proper value

    private double RETRACT_TARGET = 3.5;
   
    public void testOnlyRunAtSpeed(double pSpeed)
    {
        mExtendoMotor.set(pSpeed);
    }

    public double getEncoderReading()
    {
        return mExtendoEncoder.getPosition();
    }

    private CANSparkMax CreateExtendoArmsMotor(int pDeviceID, boolean pIsInverted)
    {
        CANSparkMax returnValue = new CANSparkMax(pDeviceID, MotorType.kBrushless);
        returnValue.setSmartCurrentLimit(39);
        returnValue.setIdleMode(IdleMode.kBrake);
        returnValue.setInverted(pIsInverted);

        returnValue.setSoftLimit(SoftLimitDirection.kForward, 68);
        returnValue.setSoftLimit(SoftLimitDirection.kReverse, 3);
        returnValue.enableSoftLimit(SoftLimitDirection.kForward, true);
        returnValue.enableSoftLimit(SoftLimitDirection.kReverse, true);
        

        returnValue.getPIDController().setP(0.05);
        returnValue.getPIDController().setI(0.0001);
        returnValue.getPIDController().setD(0.00);
        returnValue.getPIDController().setFF(0.00);
        returnValue.getPIDController().setIZone(2);

        returnValue.getPIDController().setOutputRange(-1, 1);

        returnValue.burnFlash();
        return returnValue;
    }
    
    public double getCurrentDraw()
    {
        return mExtendoMotor.getOutputCurrent();
    }

    public boolean GetTrapLimitSwitch()
    {
        return mTrapIntakeLimitSwitch.isPressed();
    }



    // Constructor
    public ExtendoArm() {
        mExtendoMotor =  CreateExtendoArmsMotor(7,false);
        mExtendoEncoder = mExtendoMotor.getEncoder();
        mExtendoEncoder.setPositionConversionFactor(1.0);
        mExtendoMotor.burnFlash();

        mTrapIntakeLimitSwitch = mExtendoMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        setSmartMotionInitial();
    }

    /*ATS everything must be done with smart motion now */
    /*
    public void testOnlyPIDToPosition(double pPositon)
    {
        goToPosition(pPositon);
    }

    private double mLastTarget = 0;
    
    
    public void goToPosition(double pPosition)
    {
        mLastTarget = pPosition;
        mExtendoMotor.getPIDController().setReference(pPosition, ControlType.kPosition);
    }
    */
    
    TrapezoidProfile.Constraints mExtendoProfileConstraints = new TrapezoidProfile.Constraints(160,600);
    TrapezoidProfile.State mExtendoProfileGoal;
    TrapezoidProfile.State mExtendoProfileSetpoint;     
    double mLastSetPosition = RETRACT_TARGET;
    public void continousSmartPositionUpdate()
    {
        double kDt = 0.005;

        mExtendoProfileGoal = new TrapezoidProfile.State(mLastSetPosition, 0);
        TrapezoidProfile profile = new TrapezoidProfile(mExtendoProfileConstraints, mExtendoProfileGoal, mExtendoProfileSetpoint);
        mExtendoProfileSetpoint = profile.calculate(kDt);
        SmartDashboard.putNumber("ExtendoArmProfileSetPoint", mExtendoProfileSetpoint.position);
        SmartDashboard.putNumber("ExtendoArmProfileVelocity", mExtendoProfileSetpoint.velocity);
        mExtendoMotor.getPIDController().setReference(mExtendoProfileSetpoint.position, ControlType.kPosition);
    }
    
    private void setSmartMotionInitial()
    {
        mExtendoProfileSetpoint = new TrapezoidProfile.State(mExtendoEncoder.getPosition(), 0);
    }

    public void smartSetPosition(double pPosition)
    {
        mLastSetPosition = pPosition;   
    }

    public void smartSetLowGoalTarget()
    {
        mLastSetPosition = LOW_GOAL_TARGET;   
    }

    public void smartSetRetractTarget()
    {
        mLastSetPosition = RETRACT_TARGET;   
    }

    // Extend arm until it gets to LOW_GOAL_EXTENSION_TARGET
    public void lowGoalExtension() 
    {
        smartSetLowGoalTarget();
        
    }

    private boolean hasReachedPosition(double pPosition)
    {
        return Math.abs(mExtendoEncoder.getPosition() - pPosition) < 0.5; //rotations of motor now...
    }

    public boolean hasReachedLowGoal()
    {
        return hasReachedPosition(LOW_GOAL_TARGET);
    }


     // Retract arm until it gets to RETRACT_TARGET
     public void retract() 
     {
        smartSetRetractTarget();
    }
    

    public void stopMotor()
    {
        mExtendoMotor.stopMotor();
    }


    public void logExtendoArm()
    {
        SmartDashboard.putNumber("ExtendoArm-Output", mExtendoMotor.getAppliedOutput());
        SmartDashboard.putNumber("ExtendoArm-Current", mExtendoMotor.getOutputCurrent());
        SmartDashboard.putNumber("ExtendoArm-Position", mExtendoEncoder.getPosition());
        SmartDashboard.putNumber("ExtendoArm-TargetPosition", mLastSetPosition);
    
    }
}