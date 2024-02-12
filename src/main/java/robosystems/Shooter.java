package robosystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

    Shooter() // initialization method
    {
        mAnglerMotor = new CANSparkFlex(0, MotorType.kBrushless);

        mPIDController = mAnglerMotor.getPIDController();
        
        // mPIDController.setP(kP);
        // mPIDController.setI(kI);
        // mPIDController.setD(kD);
        // mPIDController.setIZone(0); // don't know what these two do 
        // mPIDController.setFF(0); // ^
        // mPIDController.setOutputRange(-8, 8); // Peak output of 8 volts (?)
    
        //create a position closed-loop request, voltage output, slot 0 configs
        // mPositionVoltage = new PositionVoltage(0).withSlot(0);

        shootingFinished = true;
        mTopShooterRoller = new TalonFX(0);
        mBottomShooterRoller = new TalonFX(0);
    
    }

    

    public void setSpeed(double speed){ //set speed to integer -1.0 <= n <= 1.0
        // mRightMotor.set(speed);
        // mLeftMotor.set(speed);

        // shootingFinished = (speed == 0.0);
    }
    
    public void shootAtDefaultSpeed(){ 
        // Just cut and paste the saturday code
   
    }


    public void backingUpNoteToPreventFallingOut(){ // to prevent note escape
        mTopShooterRoller.set(mMoveBackwardsSpeed);
        mBottomShooterRoller.set(mMoveBackwardsSpeed);
    }
    
    public void setAngleBasedOnShooterMode(ShooterMode pShooterMode)
    {

    }

    public void automaticAngle() {

    }
    public void setAngle(double pDesiredPosition) //needs to be implemented
    {
        //mMotorName.setControl(mPositionVoltage.withPosition(pDesiredPosition));

    }

    public void setAngleToIntakingAngle() //needs to be implemented
    {
        //mMotorName.setControl(mPositionVoltage.withPosition(pDesiredPosition));

    }
    
}