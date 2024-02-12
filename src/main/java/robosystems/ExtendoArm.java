package robosystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ExtendoArm {
    private CANSparkMax mExtendoMotor;
    private RelativeEncoder mExtendoEncoder;

    private final int EXTENDO_CAN_ID = 7; //Use proper CAN ID
    private double LOW_GOAL_TARGET = 1000; //Replace with proper value
    private double TRAP_TARGET = 2000; //Replace with proper value
    private double RETRACT_TARGET = 0;
   
    // Constructor
    public ExtendoArm() {
        mExtendoMotor = new CANSparkMax(EXTENDO_CAN_ID, MotorType.kBrushless);
        mExtendoEncoder = mExtendoMotor.getEncoder();

        configureSparkMax(mExtendoMotor);
    }

    // Configure Extendo Motor
    private void configureSparkMax(CANSparkMax pExtendoMotor) {
        pExtendoMotor.restoreFactoryDefaults(); 
        pExtendoMotor.setSmartCurrentLimit(30); //Change value
    }

    // Extend arm until it gets to LOW_GOAL_EXTENSION_TARGET
    public void lowGoalExtension() {
        if (mExtendoEncoder.getPosition() < LOW_GOAL_TARGET){
            mExtendoMotor.set(0.2);
        } else {
            mExtendoMotor.set(0);
        }
    }

    // Extend arm until it gets to TRAP_TARGET
    public void trapExtension() {
        if (mExtendoEncoder.getPosition() < TRAP_TARGET) {
            mExtendoMotor.set(0.2);
        } else {
            mExtendoMotor.set(0);
        }
    }
     // Retract arm until it gets to RETRACT_TARGET
     public void retract() {
        if (mExtendoEncoder.getPosition() > RETRACT_TARGET) {
            mExtendoMotor.set(-0.2);
        } else {
            mExtendoMotor.set(0);
        }
    }
    
}