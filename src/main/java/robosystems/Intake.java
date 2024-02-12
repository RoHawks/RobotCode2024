package robosystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Functionality;

public class Intake {

    private TalonFX mConveyorBeltTop;
    private TalonFX mConveyorBeltBottom;
    private CANSparkMax mIntakeRoller;
    
    private double mPositionAtBreakBeam;


    public Intake(TalonFX pConveyorBeltTop, TalonFX pConveyorBeltBottom, CANSparkMax pIntakeRoller)
    {
        mConveyorBeltTop = pConveyorBeltTop;
        mConveyorBeltBottom = pConveyorBeltBottom;
        mIntakeRoller = pIntakeRoller;
        
        Functionality.configureSparkMaxStrong(pIntakeRoller);
        
        // Set to max value at the start so if the position isn't recorded then it will never backup
        mPositionAtBreakBeam = Double.MAX_VALUE;

    }

    public void recordPositionAtBreakBeam()
    {
        mPositionAtBreakBeam = mIntakeRoller.getEncoder().getPosition();
    }

    private void setSpeeds(double pBeltTopSpeed, double pBeltBottomSpeed, double pIntakeRollerSpeed)
    {
        mConveyorBeltTop.set(pBeltTopSpeed);
        mConveyorBeltBottom.set(pBeltBottomSpeed);
        mIntakeRoller.set(pIntakeRollerSpeed);
    }

    private void setSpeeds(double pSpeed)
    {
        setSpeeds(pSpeed, pSpeed, pSpeed);
    }

    public void setToNormalIntakingSpeed() 
    {
        setSpeeds(Constants.NORMAL_INTAKING_SPEED_CONVEYORS, 
                  Constants.NORMAL_INTAKING_SPEED_CONVEYORS, 
                  Constants.NORMAL_INTAKING_SPEED_INTAKE_ROLLER);
    }
    public void setWholeIntakeSpeed(double pSpeed) 
    {
        setSpeeds(pSpeed);
    }
    public void setToHoldingSpeed() 
    {
        setSpeeds(0,0, Constants.INTAKE_ROLLERS_GENTLE_ROLLING_BACKWARDS_SPEED); 
    }
    public void setToEjectingSpeed()
    {
	    setSpeeds(Constants.EJECTING_SPEED,Constants.EJECTING_SPEED, Constants.EJECTING_SPEED); 
    }

    public boolean hasConveyorFinishedBackingUp()
    {
        double motorPosition = mConveyorBeltTop.getPosition().getValueAsDouble();
        double positionChangeFromTheBreakBeam = motorPosition - mPositionAtBreakBeam;
        
        SmartDashboard.putNumber("Conveyors Position Change From BreakBeam", positionChangeFromTheBreakBeam);
    
        
        if (positionChangeFromTheBreakBeam > Constants.ROTATIONS_FOR_CONVEYORS_TO_BACK_UP_TO)
        { 
            return true;
        }
        else
        {
            return false;
        } 
    }

    public void setConveyorToBackupSpeed() 
    { 
        setSpeeds(Constants.INTAKE_CONVEYORS_BACKUP_SPEED, 
                  Constants.INTAKE_CONVEYORS_BACKUP_SPEED, 
                  Constants.INTAKE_ROLLERS_GENTLE_ROLLING_BACKWARDS_SPEED);

        
        
    }
}
/* 

public class Intake {

    

    private final int LIMIT_INTAKING = 20;
    private final int LIMIT_HOLDING = 20;
    private final int LIMIT_SHOOTING_LOW = 40;
    private final int LIMIT_SHOOTING_HIGH = 60;

    private final double DEFAULT_TOP_SPEED = 1.0;
    private final double DEFAULT_BOTTOM_SPEED = 1.0;
    private final double DEFAULT_MOTOR_SPEED = 1.0;

    public Intake(TalonFX pRollerBeltTop, TalonFX pRollerBeltBottom, CANSparkMax pRollerMotor){
        mRollerBeltTop = pRollerBeltTop;
        mRollerBeltBottom = pRollerBeltBottom;
        mRollerMotor = pRollerMotor;

        basePosition = mRollerMotor.getEncoder().getPosition();

        configureSparkMax(mRollerMotor);
    }

    public static void configureSparkMax(CANSparkMax pSparkMax){
        pSparkMax.restoreFactoryDefaults(); 
        pSparkMax.setSmartCurrentLimit(LIMIT_HOLDING);    
        pSparkMax.setOpenLoopRampRate(0.5);
        pSparkMax.burnFlash();
    }

    private void setSpeeds(double pMotorSpeed, double pBeltTopSpeed, double pBeltBottomSpeed){
        mRollerMotor.set(pMotorSpeed);
        mRollerBeltTop.set(pBeltTopSpeed);
        mRollerBeltBottom.set(pBeltBottomSpeed);
    }

    public void turnOn(){
        mIntakeActive = true; //Acts as an extra safety measure in case things get awry, makes sure the intake is actually turned on before doing anything
        setSpeeds(DEFAULT_TOP_SPEED, DEFAULT_BOTTOM_SPEED, DEFAULT_MOTOR_SPEED);
    }  
    public void turnOff(){
        mIntakeActive = false;
        setSpeeds(0, 0, 0);
    }

    public void setWholeIntakeSpeed(){ //A default version of the above command that takes no parameters, just using set default speeds (most likely what we'll be doing most of the time)
        if(mIntakeActive){
            setSpeeds(DEFAULT_TOP_SPEED, DEFAULT_BOTTOM_SPEED, DEFAULT_MOTOR_SPEED);
        }
    }

    public void ejectStuckNote(){
        if(mIntakeActive){
	    setSpeeds(EJECTING_SPEED, EJECTING_SPEED, EJECTING_SPEED);
        }
    }

    public boolean backupRollers(){
        double motorPosition = mRollerMotor.getEncoder().getPosition();
	double motorVelocity = mRollerMotor.getEncoder().getVelocity();
        if(motorVelocity != 0 || motorPosition != basePosition){ //Obviously this code can be shortened to return(condition) but I'm leaving it expanded for extra readability
            return false;
        }else{
            return true;
        }
    }
}
}

*/