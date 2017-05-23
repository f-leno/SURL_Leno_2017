/*
 * Reward Function for the backwards view RS
 *
 * Author Maxwell J Svetlik
 */

import RSGridWorld.*;

import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.core.State;
import burlap.oomdp.core.ObjectInstance;

import java.util.List;
public class BVRSRewardFunction implements RewardFunction {

	public double pr;
	private BVRS bvrs;
	
	public BVRSRewardFunction(double problemReward, BVRS bvrs){
		this.pr = problemReward;
		this.bvrs = bvrs;
	}
	
	@Override
	public double reward(State s, GroundedAction a, State sprime) {
		List<ObjectInstance> dom = s.getObjectsOfClass(BVRS.CLASSDOMAIN);
		if(dom.get(0).getBooleanValForAttribute(BVRS.ATTCHOSETERMACTION)){
			if(bvrs.getRSListSize() < 1)
				return pr;
			return bvrs.getLastRS().SarsaLearningExample(100) - pr;
		}
		return -100;
	}

}
