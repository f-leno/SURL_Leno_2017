package RSGridWorld;

import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.core.State;
import burlap.oomdp.core.ObjectInstance;

import java.util.ArrayList;

//This is a modified RF derived from UniformCostRF
//Cost is uniform with the exception of certain obstacles
//like fire, which grows more negative from adjacent cells to very negative if touching the fire
public class RSGridWorldRewardFunction implements RewardFunction {
	boolean goalRwd = true;
	int object = 0;
	ArrayList<IntPair> musicList;
	RSGridWorld dom;
	public RSGridWorldRewardFunction(RSGridWorld d){
			musicList = new ArrayList<IntPair>();
			this.dom = d;
	}
	public RSGridWorldRewardFunction(boolean goalRwd, RSGridWorld d){
		this(d);
		this.goalRwd = goalRwd;
	}
	public void clearMusicList(){
			musicList.clear();
	}
	public void setGoalReward(boolean goal){
		goalRwd = goal;
	}
	
	@Override
	public double reward(State s, GroundedAction a, State sprime) {
		ObjectInstance agentPrev = sprime.getObjectsOfClass(RSGridWorld.CLASSAGENT).get(0);
		int prevax = agentPrev.getIntValForAttribute(RSGridWorld.ATTX);
		int prevay = agentPrev.getIntValForAttribute(RSGridWorld.ATTY);

		ObjectInstance agent = sprime.getObjectsOfClass(RSGridWorld.CLASSAGENT).get(0);
		int ax = agent.getIntValForAttribute(RSGridWorld.ATTX);
		int ay = agent.getIntValForAttribute(RSGridWorld.ATTY);


		for( ObjectInstance ob : sprime.getObjectsOfClass(RSGridWorld.CLASSOBSTACLE)){
			if(ob.getStringValForAttribute(RSGridWorld.ATTNAME).equals(RSGridWorld.OBSTACLEPIT)){
				int fx = ob.getIntValForAttribute(RSGridWorld.ATTX);
				int fy = ob.getIntValForAttribute(RSGridWorld.ATTY);
				if(fx == ax && fy == ay){
					return -2500;
				}
			}
			if(ob.getStringValForAttribute(RSGridWorld.ATTNAME).equals(RSGridWorld.OBSTACLEMUSIC)){
				int fx = ob.getIntValForAttribute(RSGridWorld.ATTX);
				int fy = ob.getIntValForAttribute(RSGridWorld.ATTY);
				if(fx == ax && fy == ay){
					if(goalRwd){
					IntPair temp = new IntPair(ax,ay);
					boolean alreadyFound = dom.addLocation(temp);
					}
				}
			}
		}
		int totalRwd = -1;
		for( ObjectInstance ob : sprime.getObjectsOfClass(RSGridWorld.CLASSOBSTACLE)){
			if(ob.getStringValForAttribute(RSGridWorld.ATTNAME).equals(RSGridWorld.OBSTACLEFIRE)){
				int fx = ob.getIntValForAttribute(RSGridWorld.ATTX);
				int fy = ob.getIntValForAttribute(RSGridWorld.ATTY);
				//System.out.println("fx " + fx + " fy " + fy + " ax " + ax + " ay " + ay);
				if(fx == prevax && fy == prevay)
					return -500;
				else if(
					(((fx == (prevax - 1)) || (fx == (prevax + 1))) && (fy == prevay)) 
					|| 
					(((fy == (prevay - 1)) || (fy == (prevay + 1))) && (fx == prevax)) 
					)
					return -250;
			}
		}

		ObjectInstance loc = sprime.getObjectsOfClass(RSGridWorld.CLASSLOCATION).get(0);
		int lx = loc.getIntValForAttribute(RSGridWorld.ATTX);
		int ly = loc.getIntValForAttribute(RSGridWorld.ATTY);
		if(ax == lx && ay == ly)
			if(goalRwd)
				return 200;
		return totalRwd;
	}

	public class IntPair{
		public int x;
		public int y;
		public IntPair(int x, int y){
			this.x = x;
			this.y = y;
		}
		
		@Override
		public int hashCode(){
			return this.x + 31*this.y;
		}
		
		@Override
		public boolean equals(Object other){
			if(!(other instanceof IntPair)){
				return false;
			}
			
			IntPair o = (IntPair)other;
			return this.x == o.x && this.y == o.y;
		}
	}

}
