package RSGridWorld;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.Random;
import java.util.HashSet;
import javax.management.RuntimeErrorException;

import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.QValue;
import burlap.behavior.singleagent.planning.OOMDPPlanner;
import burlap.behavior.singleagent.planning.PlannerDerivedPolicy;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.debugtools.RandomFactory;
import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.core.State;
import burlap.oomdp.core.ObjectInstance;


/**
 * This class defines a an epsilon-greedy policy over Q-values and requires a QComputable planner to be specified.
 * With probability epsilon the policy will return a random action (with uniform distribution over all possible action).
 * With probability 1 - epsilon the policy will return the greedy action. If multiple actions tie for the highest Q-value,
 * then one of the tied actions is randomly selected.
 * @author James MacGlashan
 *
 *
 * This biased policy is constructed as per [Wiewiora et al, 2003], built on top of MacGlashan's epsilonGreedy implementation
 * Maxwell J Svetlik
 *
 */
public class BiasedPolicy extends Policy implements PlannerDerivedPolicy{

	public QComputablePlanner			qplanner;
	protected double					epsilon;
	protected Random 					rand;
	protected boolean					degeneratePolicy;
	protected AbstractGroundedAction    last_action;
	protected boolean 					next = false;
	public ArrayList<HashMap<HashSet<Coordinates>, Double>>  av;
	Map<Coordinates, Boolean> adviceMap;
	String[] obstacleName;
	int[][] heatmap = new int[15][15];
	boolean 							relativeCoord = true;
	boolean 							distanceCoord 	  = false;
	protected Policy p;
	int count = 0;
	/**
	 * Initializes with the value of epsilon, where epsilon is the probability of taking a random action.
	 * @param epsilon the probability of taking a random action.
	 */
	public BiasedPolicy(double epsilon) {
		qplanner = null;
		degeneratePolicy = true;
		this.epsilon = epsilon;
		rand = RandomFactory.getMapped(0);
	}
	
	/**
	 * Initializes with the QComputablePlanner to use and the value of epsilon to use, where epsilon is the probability of taking a random action.
	 * @param planner the QComputablePlanner to use
	 * @param epsilon the probability of taking a random action.
	 */
	public BiasedPolicy(QComputablePlanner planner, double epsilon) {
		qplanner = planner;
		this.epsilon = epsilon;
		degeneratePolicy = true;
		rand = RandomFactory.getMapped(0);
		count = 0;
	}

	public BiasedPolicy(double epsilon, ArrayList<HashMap<HashSet<Coordinates>, Double>> agentValueMap, String[] obstacleName){
		av = new ArrayList<HashMap<HashSet<Coordinates>, Double>>(agentValueMap);
		this.obstacleName = obstacleName;
		this.qplanner = qplanner;
		this.epsilon = epsilon;
		degeneratePolicy = false;
		rand = RandomFactory.getMapped(0);
		adviceMap = new HashMap<Coordinates, Boolean>();
		count = 0;
	}
	
	public BiasedPolicy(double epsilon, final ArrayList<HashMap<HashSet<Coordinates>, Double>> agentValueMap, String obstacleName){
		this(epsilon, agentValueMap, new String[] {obstacleName});
	}

	/**
	 * Returns the epsilon value, where epsilon is the probability of taking a random action.
	 * @return the epsilon value
	 */
	public double getEpsilon() {
		return epsilon;
	}

	/**
	 * Sets the epsilon value, where epsilon is the probability of taking a random action.
	 * @param epsilon the probability of taking a random action.
	 */
	public void setEpsilon(double epsilon) {
		this.epsilon = epsilon;
	}

	@Override
	public void setPlanner(OOMDPPlanner planner){
		
		if(!(planner instanceof QComputablePlanner)){
			throw new RuntimeErrorException(new Error("Planner is not a QComputablePlanner"));
		}
		
		this.qplanner = (QComputablePlanner)planner;
	}

	public void setDegenerate(boolean degen){
		this.degeneratePolicy = degen;
	}

	public double distance(int sx, int sy, int tx, int ty){
		return Math.sqrt(Math.pow(tx - sx, 2) + Math.pow(ty-sy,2));
	}

	/*
	* Note that this is the same scheme and function used in RSGridWorldPotentialFunction.java
	*/
	public double biasedValue(State s, AbstractGroundedAction aga){
		ObjectInstance a = s.getObjectsOfClass(RSGridWorld.CLASSAGENT).get(0);
		List<ObjectInstance> obst = s.getObjectsOfClass(RSGridWorld.CLASSOBSTACLE);
		ObjectInstance locs = s.getObjectsOfClass(RSGridWorld.CLASSLOCATION).get(0);

		String xa = RSGridWorld.ATTX;
		String ya = RSGridWorld.ATTY;
		String lt = RSGridWorld.ATTLOCTYPE;
		String na = RSGridWorld.ATTNAME;
		double distance = Double.MAX_VALUE;
		int ax = a.getIntValForAttribute(xa);
		int ay = a.getIntValForAttribute(ya);
		int ox,oy;
		int index = 0;
		boolean usedAS = false;
		double pot = 0.;
		int xdelta = 0;
		int ydelta = 0;
		double cumulative_pot = 0;
		double min = Double.MAX_VALUE;
		double max = Double.MIN_VALUE;

		/*
		* Check all obstacles positions, if they've been seen before (relative to the agent) use the value stored in the agent map
		*/
		if(relativeCoord){
			HashSet<Coordinates> locations = new HashSet<Coordinates>();
			Coordinates temp;
			for(ObjectInstance l : obst){
				if(!l.getStringValForAttribute(na).equals(RSGridWorld.OBSTACLEMUSIC)){
				ox = l.getIntValForAttribute(xa);
				oy = l.getIntValForAttribute(ya);
				if(distanceCoord){
					double dist = distance(ax,ay,ox,oy);
					temp = new Coordinates(dist,dist, aga.actionName(),l.getStringValForAttribute(na));
				}
				else
					temp = new Coordinates(ax-ox,ay-oy, aga.actionName(),l.getStringValForAttribute(na));

				locations.add(temp);
				}
			}
			HashSet<Coordinates> positions = new HashSet<Coordinates>();
			ox = locs.getIntValForAttribute(xa);
			oy = locs.getIntValForAttribute(ya);
			if(distanceCoord){
				double dist = distance(ax,ay,ox,oy);
				temp = new Coordinates(dist,dist, aga.actionName(),RSGridWorld.OBSTACLEGOAL);
			}
			else
				temp = new Coordinates(ax-ox,ay-oy, aga.actionName(),RSGridWorld.OBSTACLEGOAL);
			positions.add(temp);
			int count = 0;

			for(int i = 0; i < av.size(); i++){ //although maybe not clear, we have av.length pot functions
				temp = new Coordinates(0,0);
				//check for a goal tuple
				if(av.get(i).containsKey(positions)){
					pot += av.get(i).get(positions);
					//System.out.println("Adding " + av.get(i).get(positions) + " due to goal potential");
				}
				HashMap<HashSet<Coordinates>, Double> tuple = av.get(i);
				for(HashSet c : tuple.keySet()){
					if(locations.containsAll(c)){
						count++;
						pot+= tuple.get(c);
						//System.out.println("Adding " + tuple.get(c));
					}
				}
				
				/*
				 * Choose how to merge potentials. Heuristic policy is set in RSGridWorld.java
				 */
				if(RSGridWorld.HEURISTIC.equals("max")){
					if(pot > max){
						max = pot;
					}
				}
				else if(RSGridWorld.HEURISTIC.equals("min")){
					if(pot < min)
						min = pot;
				}
				else if(RSGridWorld.HEURISTIC.equals("average")){
					cumulative_pot += pot;
				}
			}
		}
		else{
			for(int i = 0; i < av.size(); i++){ //although maybe not clear, we have obstacleName.length pot functions
				HashSet<Coordinates> positions = new HashSet<Coordinates>();
				ox = locs.getIntValForAttribute(xa);
				oy = locs.getIntValForAttribute(ya);
				Coordinates temp = new Coordinates(ax,ay, aga.actionName(),RSGridWorld.OBSTACLEGOAL);
				positions.add(temp);
				if(av.get(i).containsKey(positions)){
					pot += av.get(i).get(positions);
				}

				positions.clear();
				ax = a.getIntValForAttribute(xa);
				ay = a.getIntValForAttribute(ya);
				temp = new Coordinates(ax,ay, aga.actionName());
				HashMap<HashSet<Coordinates>, Double> tuple = av.get(i);
				positions.add(temp);
				if(tuple.containsKey(positions)){
					pot+= tuple.get(positions);
				}
				
				/*
				 * Choose how to merge potentials. Heuristic policy is set in RSGridWorld.java
				 */
				if(RSGridWorld.HEURISTIC.equals("max")){
					if(pot > max){
						max = pot;
					}
				}
				else if(RSGridWorld.HEURISTIC.equals("min")){
					if(pot < min)
						min = pot;
				}
				else if(RSGridWorld.HEURISTIC.equals("average")){
					cumulative_pot += pot;
				}
			}
		}
		if(RSGridWorld.HEURISTIC.equals("max"))
			return max;
		if(RSGridWorld.HEURISTIC.equals("min"))
			return min;
		if(RSGridWorld.HEURISTIC.equals("average")){
			//System.out.println("Got " + cumulative_pot);
			return cumulative_pot;
		}
		System.out.println("Warning, no heuristic set!!!");
		return Integer.MIN_VALUE;
	}
	public AbstractGroundedAction getBiasedAction(State s){
		//if(!next){
			last_action = getAction(s);
			next = true;
		//}
		return last_action;
	}
	
	@Override
	public AbstractGroundedAction getAction(State s) {
		if(next){ //then next action has already been called and it should return what was last computed;
			next = false;
			return last_action;
		}
		ObjectInstance a = s.getObjectsOfClass(RSGridWorld.CLASSAGENT).get(0);
		
		String xa = RSGridWorld.ATTX;
		String ya = RSGridWorld.ATTY;
		int ax = a.getIntValForAttribute(xa);
		int ay = a.getIntValForAttribute(ya);
		
		List<QValue> qValues = this.qplanner.getQs(s);
		
		
		double roll = rand.nextDouble();
		if(roll <= epsilon){
			int selected = rand.nextInt(qValues.size());
			//System.out.println("Rolled the dice! Going " + qValues.get(selected).a.translateParameters(qValues.get(selected).s, s).actionName());
			return qValues.get(selected).a;//.translateParameters(qValues.get(selected).s, s);
		}
		
		
		List <QValue> maxActions = new ArrayList<QValue>();
		maxActions.add(qValues.get(0));
		double maxQ = qValues.get(0).q;
		if(!degeneratePolicy){
			maxQ += biasedValue(s,qValues.get(0).a);
			//System.out.println("Using biased QValue: " + maxQ + " instead of unbiased QValue: " + qValues.get(0).q + " for action: " + qValues.get(0).a.actionName());		
		}
		QValue q;
		for(int i = 1; i < qValues.size(); i++){
			if(!degeneratePolicy){
				Coordinates temp = new Coordinates(ax,ay,qValues.get(i).a.actionName());
				double biasedval = biasedValue(s,qValues.get(i).a);
				//if(biasedval != qValues.get(i).q){
				//if(!adviceMap.containsKey(temp)){ 
					q = new QValue(s,qValues.get(i).a, biasedval + qValues.get(i).q);
					//adviceMap.put(temp, true);
				//}
				//else
				//	q = new QValue(s,qValues.get(i).a, qValues.get(i).q); //qValues.get(i) + biasedValue(s);

				//q = new QValue(s,qValues.get(i).a, biasedValue(s,qValues.get(i).a) + qValues.get(i).q); //qValues.get(i) + biasedValue(s);
				//System.out.println("Using biased QValue: " + q.q + " instead of unbiased QValue: " + qValues.get(i).q + " for action: " + q.a.actionName());
			}
			else {
				q = qValues.get(i);
				//System.out.println("Using unbiased QValue: " + q.q);
			}
			if(q.q == maxQ){
				maxActions.add(q);
			}
			else if(q.q > maxQ){
				maxActions.clear();
				maxActions.add(q);
				maxQ = q.q;
			}
		}
		int selected = rand.nextInt(maxActions.size());
		//return translated action parameters if the action is parameterized with objects in a object identifier indepdent domain
		

		//System.out.println("I'm at: " + ax + ", " + ay + " going " + maxActions.get(selected).a.actionName() + 
		//	"because of the maxValue : " + maxQ + "\n");


		return maxActions.get(selected).a;//.a.translateParameters(maxActions.get(selected).s, s);
	}

	@Override
	public List<ActionProb> getActionDistributionForState(State s) {
		
		List<QValue> qValues = this.qplanner.getQs(s);
		
		List <ActionProb> dist = new ArrayList<Policy.ActionProb>(qValues.size());
		double maxQ = Double.NEGATIVE_INFINITY;
		int nMax = 0;
		for(QValue q : qValues){
			if(!degeneratePolicy){
				double biasedval = biasedValue(s, q.a);
				if((q.q + biasedval) > maxQ){
					maxQ = q.q + biasedval;
					nMax = 1;
				}
				else if(q.q + biasedval == maxQ){
					nMax++;
				}
			}
			else{
				if((q.q) > maxQ){
					maxQ = q.q;
					nMax = 1;
				}
				else if(q.q == maxQ){
					nMax++;
				}
			}
			ActionProb ap = new ActionProb(q.a.translateParameters(q.s, s), this.epsilon*(1. / qValues.size()));
			dist.add(ap);
		}
		for(int i = 0; i < dist.size(); i++){
			QValue q = null;
			if(!degeneratePolicy)
				q = new QValue(s,qValues.get(i).a, biasedValue(s,qValues.get(i).a) + qValues.get(i).q);
			else
				q = new QValue(s,qValues.get(i).a, qValues.get(i).q);

			if(q.q == maxQ){
				dist.get(i).pSelection += (1. - this.epsilon) / nMax;
			}
		}
		
		
		return dist;
	}

	@Override
	public boolean isStochastic() {
		return true;
	}
	
	@Override
	public boolean isDefinedFor(State s) {
		return true; //can always find q-values with default value
	}

}
