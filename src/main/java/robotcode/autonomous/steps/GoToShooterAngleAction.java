package robotcode.autonomous.steps;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import robosystems.Intake;
import robosystems.Shooter;
import robotcode.autonomous.AAction;
import states.JoystickControlsWithSwerve;
import states.States;

public class GoToShooterAngleAction extends AAction{

    private Shooter mShooter;
    private ArrayList<Pair<Double, Long>> mListOfPairs;


    public GoToShooterAngleAction(Shooter pShooter, ArrayList<Pair<Double, Long>> pListOfPairs)
    {
        mShooter = pShooter;
        mListOfPairs = pListOfPairs;
    }

    @Override
    public boolean Run() 
    {
        int idxOfCommand = -1;
        boolean hasSetAngleInThisIteration = false;
        for (int i = 0; i < mListOfPairs.size(); i++)
        {
            Pair<Double,Long> p = mListOfPairs.get(i);
            if (!HasTimeElapsed(p.getSecond()))
            {
                mShooter.setAngle(p.getFirst());
                hasSetAngleInThisIteration = true;
                idxOfCommand = i;
                break;
            }
        }
        if(!hasSetAngleInThisIteration )
        {
            mShooter.setAngle(mListOfPairs.get(mListOfPairs.size() - 1).getFirst());
        }


        if(idxOfCommand == mListOfPairs.size() - 1 && mShooter.checkIfAnglerIsCloseEnough())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    @Override
    public void EndAction() {
        
    }
    
    
}
