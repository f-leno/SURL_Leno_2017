package RSGridWorld;
import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.core.State;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.QValue;
import burlap.behavior.learningrate.LearningRate;
import burlap.oomdp.core.ObjectInstance;

import java.util.HashSet;
import java.util.HashMap;
import java.util.Map;
public class RSGridWorldLearningRate implements LearningRate {

	public double learningRate = 0.1;
	public Map<HashSet<Coordinates>, Double> av;
	public QLearning agent;
	public HashMap<HashSet<Coordinates>, Double> occ = new HashMap<HashSet<Coordinates>, Double>();
	public RSGridWorldLearningRate(Map<HashSet<Coordinates>, Double> av){
		this.av = av;
	}
	
	/**
	 * Constructs a constant learning rate for the given value
	 * @param learningRate the constant learning rate to use
	 */
	public RSGridWorldLearningRate(Double learningRate){
		this.learningRate = learningRate;
	}

	/*
	This is REQUIRED for value initialization
	*/
	public void setAgent(QLearning agent){
		this.agent = agent;
	}
	
	@Override
	public double peekAtLearningRate(State s, AbstractGroundedAction ga) {
		return this.learningRate;
	}

	@Override
	public double pollLearningRate(int agentTime, State s, AbstractGroundedAction ga) {
		ObjectInstance a = s.getObjectsOfClass(RSGridWorld.CLASSAGENT).get(0);
		ObjectInstance locs = s.getObjectsOfClass(RSGridWorld.CLASSLOCATION).get(0);
		int ax = a.getIntValForAttribute("x");
		int ay = a.getIntValForAttribute("y");
		int lox = locs.getIntValForAttribute("x");
		int loy = locs.getIntValForAttribute("y");
		HashSet<Coordinates> t = new HashSet<Coordinates>();
		Coordinates temp = new Coordinates((double)ax, (double)ay,ga.actionName(),RSGridWorld.OBSTACLEGOAL);
		t.add(temp);
		if(occ.containsKey(t)){
			double alpha = occ.get(t);
			if(alpha > 0.1)
				occ.put(t,alpha/2);
			return alpha;
		}
		else{
			occ.put(t,0.75);
			//System.out.println("Initializing " + (ax - lox) + " , " + (ay - loy) + " " + ga.actionName() + " with " + av.get(t));
			return 1;
		}
	}

	@Override
	public void resetDecay() {
		//no change needed
	}

	@Override
	public double peekAtLearningRate(int featureId) {
		return this.learningRate;
	}

	@Override
	public double pollLearningRate(int agentTime, int featureId) {
		return this.learningRate;
	}

}