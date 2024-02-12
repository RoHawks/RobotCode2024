 package states;

import robosystems.Arm;
import robosystems.Intake;
import universalSwerve.SwerveDrive;

public class GameStartState extends AState
{
    private SwerveDrive mSwerveDrive;
    private Intake mIntake;
    private Arm mArm;

    private boolean mHasCompletedGameStart;
    
    public GameStartState(
        SwerveDrive pSwerveDrive,
        Intake pIntake,
        Arm pArm)
        {
            mSwerveDrive = pSwerveDrive;
            mIntake = pIntake;
            mArm = pArm;
            
        }
    
     @Override
     protected String GetName() {
        
         return "GameStart";
     }

     @Override 
     public void EnterState(Object pEntryParameter)
     {
         super.EnterState(pEntryParameter);
        
     }

    @Override
    protected NextStateInfo Run() 
    {
        if(mHasCompletedGameStart)        
        {
            //this will be true when we transition from auto to teleop
            //and we don't want to really do the game start routines.
            return new NextStateInfo(States.Intaking);
        }
        else
        {
            mIntake.deploy();
            mSwerveDrive.StopEverything();
            mArm.NormalHoldingStill();    
            //mSwerveDrive.SetGyroscopeCurrentAngle(180); moving this to Robot.java
            mHasCompletedGameStart = true;
            return new NextStateInfo(States.Intaking);
        }
    }

     @Override
    protected States GetState() {
         return States.GameStart;
    }
    
}
