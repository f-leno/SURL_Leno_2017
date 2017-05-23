package RSGridWorld;

import burlap.behavior.singleagent.shaping.ShapedRewardFunction;
import burlap.oomdp.core.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.behavior.singleagent.shaping.potential.PotentialFunction;
import burlap.oomdp.core.ObjectInstance;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
/**
 * This class is used to implement Potential-based reward shaping [1] which is guaranteed to preserve the optimal policy. This class
 * requires a {@link PotentialFunction} and the discount being used by the MDP. The additive reward is defined as:
 * d * p(s') - p(s)
 * where d is this discount factor, s' is the most recent state, s is the previous state, and p(s) is the potential of state s.
 * 
 * 
 * 1. Ng, Andrew Y., Daishi Harada, and Stuart Russell. "Policy invariance under reward transformations: Theory and application to reward shaping." ICML. 1999.
 * 
 * @author James MacGlashan
 *



 Edits: reformed additiveReward - mjs
 */
public class RSPotentialShapedRF extends ShapedRewardFunction {

	
	/**
	 * The potential function that can be used to return the potential reward from input states.
	 */
	public ArrayList<PotentialFunction>			potentialFunction;
	HashMap<String, Double> last_used = new HashMap<String,Double>();
	public RewardFunction base;
	public double lastpot = 0.0;
	public double salastpot = 0.0;
	public AbstractGroundedAction lasta;

	/**
	 * The discount factor the MDP (required for this to shaping to preserve policy optimality)
	 */
	public double					discount;
	 double sum = 0.0;
	
	/**
	 * Initializes the shaping with the objective reward function, the potential function, and the discount of the MDP.
	 * @param baseRF the objective task reward function.
	 * @param potentialFunction the potential function to use.
	 * @param discount the discount factor of the MDP.
	 */
	public RSPotentialShapedRF(RewardFunction baseRF, PotentialFunction[] potentialFunction, double discount) {
		super(baseRF);
		this.potentialFunction = new ArrayList<PotentialFunction>();
		base = baseRF;
		for(int i = 0; i < potentialFunction.length; i++){
			this.potentialFunction.add(potentialFunction[i]);
		}
		this.discount = .99;
	}
	public RSPotentialShapedRF(RewardFunction baseRF, ArrayList<PotentialFunction> potentialFunction, double discount) {
		super(baseRF);
		this.potentialFunction = potentialFunction;
		this.discount = discount;
		this.base = baseRF;
	}
	public RSPotentialShapedRF(RSPotentialShapedRF old){
		this(old.base, old.potentialFunction, old.discount);
	}
	public void clearBaseOccGrid(){
		((RSGridWorldRewardFunction) baseRF).clearMusicList();
	}
	public void reset(){
		for(int i = 0; i < potentialFunction.size(); i++)
			((RSGridWorldPotentialFunction)potentialFunction.get(i)).reset();
		lastpot = 0.0;
		salastpot = 0.0;
		last_used.clear();
	}
	public double setLast(double last){
		double old = this.lastpot;
		this.lastpot = last;
		return old;
	}


	@Override
	public double additiveReward(State s, GroundedAction a, State sprime) {
		double additiveRwd = 0;
		boolean gotSA = false;
		double max = Double.MIN_VALUE;
		double min = Double.MAX_VALUE;
		ObjectInstance a_last = s.getObjectsOfClass(RSGridWorld.CLASSAGENT).get(0);
		ObjectInstance a_cur = sprime.getObjectsOfClass(RSGridWorld.CLASSAGENT).get(0);
		List<ObjectInstance> obst = s.getObjectsOfClass(RSGridWorld.CLASSOBSTACLE);
		int xprime = a_cur.getIntValForAttribute(RSGridWorld.ATTX);
		int yprime = a_cur.getIntValForAttribute(RSGridWorld.ATTY);
		int x = a_last.getIntValForAttribute(RSGridWorld.ATTX);
		int y = a_last.getIntValForAttribute(RSGridWorld.ATTY);
		double curPot = 0;
		double prevPot = 0;
		double pot = 0.;
		for(PotentialFunction pf : this.potentialFunction){

			if(!gotSA && ((RSGridWorldPotentialFunction) pf).getStateAction()){
				prevPot = lastpot;
				curPot = ((RSGridWorldPotentialFunction)pf).potentialValue(s,a);
				lastpot = new Double(curPot);
				lasta = a.copy();
				//if(Math.abs(x - xprime) + Math.abs(y-yprime) > 1)
				//	prevPot = 0;
				gotSA = true;
			}else if (!((RSGridWorldPotentialFunction)pf).getStateAction()){
				if(last_used.containsKey(((RSGridWorldPotentialFunction)pf).obstacleName))
					prevPot = last_used.get(((RSGridWorldPotentialFunction)pf).obstacleName);
				else
					prevPot = 0;
				//prevPot = salastpot;
				//prevPot = ((RSGridWorldPotentialFunction)pf).potentialValue(s);
				curPot = ((RSGridWorldPotentialFunction)pf).potentialValue(sprime);
				last_used.put(((RSGridWorldPotentialFunction)pf).obstacleName, new Double(curPot));
				salastpot = new Double(curPot);
				//System.out.println("Curpot: " + curPot + " lastpot: " + prevPot);
			}
			if(Math.abs(x - xprime) + Math.abs(y-yprime) > 1)
				prevPot = 0;
			boolean inpit = false;
			boolean inpitprime = false;

			pot = this.discount * curPot - prevPot;
			
			additiveRwd += pot;

			sum += additiveRwd;
			pot = 0.;
		}
		if(RSGridWorld.HEURISTIC.equals("max"))
			return max;
		if(RSGridWorld.HEURISTIC.equals("min"))
			return min;
		if(RSGridWorld.HEURISTIC.equals("average")){
			return additiveRwd;
		}
		
		
		System.out.println("Warning, no heuristic set!!!");
		return Integer.MIN_VALUE;
	}
}

