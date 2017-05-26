package RSGridWorld;

import burlap.oomdp.core.State;
import burlap.oomdp.core.ObjectInstance;

import java.util.HashSet;
import java.util.ArrayList;
import java.util.List;


/*
 * This class provides methods for comparing Tasks directly.
 * As input, the constructor takes the filepath to the tasks and builds domain objects.
 *
 * This is the primary class for holding the applicability and statespace functions.
 */
public class CompareTask{

    RSGridWorld                 source;
    RSGridWorld                 target;
    State 						initState_source;
    State 						initState_target;
    boolean 					includeGoal;

	public CompareTask(String task1, String nextTask){
        DomainToFile dtf = new DomainToFile();
        dtf.parseFile(task1);
        source = dtf.getDomain();
        initState_source = dtf.getState();

        dtf = new DomainToFile();
        dtf.parseFile(nextTask);
        target = dtf.getDomain();
        initState_target = dtf.getState();
        includeGoal = true;
        if(task1.contains("goal") || nextTask.contains("target"))
        	includeGoal = false;//true;
	}
	/*
	 * Due to the representation of the task in-file, you must specify if the goal should
	 * be included as a feature in the source.
	 */
	public CompareTask(String task1, String nextTask, boolean includeGoal){
		this(task1, nextTask);
		this.includeGoal = includeGoal;
	}

	public CompareTask(){};

	/*
	 * This function returns the number of states that would be fired in a target task if a potential function
	 * was learned and then transfered.
	 *
	 */
	public int getApplicability(){
		List<ObjectInstance> source_obst = initState_source.getObjectsOfClass(RSGridWorld.CLASSOBSTACLE);
		List<ObjectInstance> source_locs = initState_source.getObjectsOfClass(RSGridWorld.CLASSLOCATION);
		HashSet<ArrayList<Coordinates> > source_pot = new HashSet<ArrayList<Coordinates> >();

		/*
		 * Grab relational information about the source
		 */
		int width = source.getWidth();
		if(isSourceCoarse())
			width *= width;

		for(int i = 0; i < width; i++){
			for(int j = 0; j < source.getHeight(); j++){
				ArrayList<Coordinates> distances = new ArrayList<Coordinates>();
				for(ObjectInstance l : source_obst){
					if(l.getStringValForAttribute(RSGridWorld.ATTNAME).equals(RSGridWorld.OBSTACLEPIT) ||
									l.getStringValForAttribute(RSGridWorld.ATTNAME).equals(RSGridWorld.OBSTACLEFIRE)){
						Coordinates observation = new Coordinates(i - l.getIntValForAttribute(RSGridWorld.ATTX), j - l.getIntValForAttribute(RSGridWorld.ATTY), 
							l.getStringValForAttribute(RSGridWorld.ATTNAME), true);
						distances.add(observation);
					}
				}
				if(includeGoal){
					for(ObjectInstance l : source_locs){
						Coordinates observation = new Coordinates(i - l.getIntValForAttribute(RSGridWorld.ATTX), j - l.getIntValForAttribute(RSGridWorld.ATTY), 
							RSGridWorld.OBSTACLEGOAL, true);
						distances.add(observation);
						break;
					}
				}
				source_pot.add(distances);
			}
		}

		/*
		 * Grab relational information about the target
		 */

		List<ObjectInstance> target_obst = initState_target.getObjectsOfClass(RSGridWorld.CLASSOBSTACLE);
		List<ObjectInstance> target_locs = initState_target.getObjectsOfClass(RSGridWorld.CLASSLOCATION);
		HashSet<ArrayList<Coordinates> > target_pot = new HashSet<ArrayList<Coordinates> >();

		for(int i = 0; i < target.getWidth(); i++){
				for(int j = 0; j < target.getHeight(); j++){
					ArrayList<Coordinates> distances = new ArrayList<Coordinates>();
					for(ObjectInstance l : target_obst){
						if(l.getStringValForAttribute(RSGridWorld.ATTNAME).equals(RSGridWorld.OBSTACLEPIT) ||
										l.getStringValForAttribute(RSGridWorld.ATTNAME).equals(RSGridWorld.OBSTACLEFIRE)){
							Coordinates observation = new Coordinates(i - l.getIntValForAttribute(RSGridWorld.ATTX), j - l.getIntValForAttribute(RSGridWorld.ATTY), 
								l.getStringValForAttribute(RSGridWorld.ATTNAME), true);
							distances.add(observation);
						}
					}

					for(ObjectInstance l : target_locs){
						Coordinates observation;
						if(isSourceCoarse()){
							observation = new Coordinates(i - l.getIntValForAttribute(RSGridWorld.ATTX), i - l.getIntValForAttribute(RSGridWorld.ATTX), 
								RSGridWorld.OBSTACLEGOAL, true);
							distances.add(observation);
							observation = new Coordinates(j - l.getIntValForAttribute(RSGridWorld.ATTX), j - l.getIntValForAttribute(RSGridWorld.ATTY), 
								RSGridWorld.OBSTACLEGOAL, true);
						}

						else
							observation = new Coordinates(i - l.getIntValForAttribute(RSGridWorld.ATTX), j - l.getIntValForAttribute(RSGridWorld.ATTY), 
								RSGridWorld.OBSTACLEGOAL, true);
						distances.add(observation);
						break;
					}
					target_pot.add(distances);
				}
			}

		/*
		 * Determine applicable states
		 */

		int applicable = 0;
		
		for(ArrayList<Coordinates> tar : target_pot){
			for(ArrayList<Coordinates> src : source_pot){
				if(tar.containsAll(src))
					applicable++;
			}
		}
		//System.out.println("applies in " + applicable + " states.");
		return applicable;
	}
	/*
	 * returns whether or not the source is a course task
	 */
	public boolean isSourceCoarse(){
		if(source.getWidth() == 1 || source.getHeight() == 1)
			return true;
		return false;
	}
	/*
	 * returns whether or not the source is a course task
	 */
	public boolean isTargetCoarse(){
		if(target.getWidth() == 1 || target.getHeight() == 1)
			return true;
		return false;
	}

	/*
	 * returns the number of states in the source task
	 */
	public double getSourceSpace(){
		return (double) (source.getWidth() * source.getHeight());
	}

	/*
	 * returns the number of states in the target task
	 */
	public double getTargetSpace(){
		return (double) (target.getWidth() * target.getHeight());
	}

	/*
	 * Given a task filepath, returns an arrayList of features
	 * This includes a location (goal) if specified infile.
	 */
	public HashSet<String> getTaskFeatures(String taskFile){

		HashSet<String> res = new HashSet<String>();
		DomainToFile dtf = new DomainToFile();
        dtf.parseFile(taskFile);
        RSGridWorld dom = dtf.getDomain();
        State domState = dtf.getState();

		List<ObjectInstance> task_obst = domState.getObjectsOfClass(RSGridWorld.CLASSOBSTACLE);
		List<ObjectInstance> task_locs = domState.getObjectsOfClass(RSGridWorld.CLASSLOCATION);

        for(ObjectInstance l : task_obst){
			if(l.getStringValForAttribute(RSGridWorld.ATTNAME).equals(RSGridWorld.OBSTACLEPIT)){
				res.add(RSGridWorld.OBSTACLEPIT);
			}
			else if(l.getStringValForAttribute(RSGridWorld.ATTNAME).equals(RSGridWorld.OBSTACLEFIRE)){
				res.add(RSGridWorld.OBSTACLEFIRE);
			}
		}

		for(ObjectInstance l : task_locs){
			res.add(RSGridWorld.OBSTACLEGOAL);
			break;
		}	
		return res;
	}
}