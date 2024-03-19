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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ExtendoArm {
    private CANSparkMax mExtendoMotor;
    private RelativeEncoder mExtendoEncoder;
    private SparkLimitSwitch mTrapIntakeLimitSwitch;
    
    private double LOW_GOAL_TARGET = -3600; //Replace with proper value
    private double TRAP_TARGET = -2000; //Replace with proper value
    private double RETRACT_TARGET = -50;
   
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
        CANSparkMax returnValue = new CANSparkMax(pDeviceID, MotorType.kBrushed);
        returnValue.setSmartCurrentLimit(39);
        returnValue.setIdleMode(IdleMode.kBrake);
        returnValue.setInverted(pIsInverted);

        returnValue.setSoftLimit(SoftLimitDirection.kForward, -20);
        returnValue.setSoftLimit(SoftLimitDirection.kReverse, -3700);
        returnValue.enableSoftLimit(SoftLimitDirection.kForward, true);
        returnValue.enableSoftLimit(SoftLimitDirection.kReverse, true);
        

        returnValue.getPIDController().setP(0.0014);
        returnValue.getPIDController().setI(0.000004);
        returnValue.getPIDController().setD(0.00);
        returnValue.getPIDController().setFF(0.00);
        returnValue.getPIDController().setIZone(500);

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
        mExtendoEncoder = mExtendoMotor.getEncoder(Type.kQuadrature, 1);

        mTrapIntakeLimitSwitch = mExtendoMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        
    }

    private double mLastTarget = 0;
    public void goToPosition(double pPosition)
    {
        mLastTarget = pPosition;
        mExtendoMotor.getPIDController().setReference(pPosition, ControlType.kPosition);
    }


    // Extend arm until it gets to LOW_GOAL_EXTENSION_TARGET
    public void lowGoalExtension() 
    {
        goToPosition(LOW_GOAL_TARGET);
    }

    private boolean hasReachedPosition(double pPosition)
    {
        return Math.abs(mExtendoEncoder.getPosition() - pPosition) < 100;
    }

    public boolean hasReachedLowGoal()
    {
        return hasReachedPosition(LOW_GOAL_TARGET);
    }

    // Extend arm until it gets to TRAP_TARGET
    public void trapExtension() {
        goToPosition(TRAP_TARGET);
    }
     // Retract arm until it gets to RETRACT_TARGET
     public void retract() 
     {
        goToPosition(RETRACT_TARGET);
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
        SmartDashboard.putNumber("ExtendoArm-TargetPosition", mLastTarget);
    
    }
}