package robosystems;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;

public class ExtendoArm {
    private CANSparkMax mExtendoMotor;
    private RelativeEncoder mExtendoEncoder;
    
    private double LOW_GOAL_TARGET = -2100; //Replace with proper value
    private double TRAP_TARGET = -2800; //Replace with proper value
    private double RETRACT_TARGET = -50;
   
    

    private CANSparkMax CreateExtendoArmsMotor(int pDeviceID, boolean pIsInverted)
    {
        CANSparkMax returnValue = new CANSparkMax(pDeviceID, MotorType.kBrushed);
        returnValue.setSmartCurrentLimit(39);
        returnValue.setIdleMode(IdleMode.kBrake);
        returnValue.setInverted(pIsInverted);

        returnValue.setSoftLimit(SoftLimitDirection.kForward, -20);
        returnValue.setSoftLimit(SoftLimitDirection.kReverse, -2500);
        returnValue.enableSoftLimit(SoftLimitDirection.kForward, true);
        returnValue.enableSoftLimit(SoftLimitDirection.kReverse, true);
        

        returnValue.getPIDController().setP(0.005);
        returnValue.getPIDController().setI(0.000);
        returnValue.getPIDController().setD(0.00);
        returnValue.getPIDController().setFF(0.00);

        returnValue.burnFlash();
        return returnValue;
    }
    


    // Constructor
    public ExtendoArm() {
        mExtendoMotor =  CreateExtendoArmsMotor(7,false);
        mExtendoEncoder = mExtendoMotor.getEncoder(Type.kQuadrature, 1);
    }

    public void goToPosition(double pPosition)
    {
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
}