
/*
 * Backwards-view reward shaping framework
 *
 * Structured as an MDP (or pomdp) where actions are manipulations of the domain space
 * Reward is either time of computation (to convergence) or some inverse of the average policy reward (which would need to be normalized)
 * State represented by the non-egocentric (/agentspace/task) domain attributes
 *
 * Author Maxwell J Svetlik
 *
 */

import RSGridWorld.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Map;
import java.util.HashMap;
import java.util.Arrays;

import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.debugtools.RandomFactory;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.core.Attribute;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.ObjectClass;
import burlap.oomdp.core.ObjectInstance;
import burlap.oomdp.core.PropositionalFunction;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TransitionProbability;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.explorer.TerminalExplorer;
import burlap.oomdp.singleagent.explorer.VisualExplorer;
import burlap.oomdp.visualizer.Visualizer;

public class BVRS implements DomainGenerator{

	/**
	 * Action constants
	 */
	public static final String 			ACTIONCHANGEGRIDSIZE = "changegridsize";

	public static final String 			ACTIONCHANGEOBSTACLENUM = "changeobnum";

	public static final String 			ACTIONCHANGEOBSTACLETYPE = "changeobtype";

	public static final String 			ACTIONEXECUTETRIALS = "executetrials";

	/**
	 * Object classes
	 */
	public static final String 			CLASSDOMAIN = "domain";

	/*
	 * Attributes
	 */
	public static final String 			ATTSIZE = "size";

	public static final String 			ATTOBSTACLENUM = "obstaclenum";

	public static final String 			ATTOBSTACLETYPE = "obstacletype";

	public static final String 			ATTCHOSETERMACTION = "chosetermaction";


	protected double[][]								transitionDynamics;

	/*
	 * Defines the number of domain files in each domain folder type
	 */
	public static int NUMDOMAINS = 20;
	/*
	 * Number of types (folders of domains)
	 */
	public static int NUMDOMAINTYPE = 3;
	/*
	 * List holding the domains to be used as curriculum
	 */
	public ArrayList<RewardShape> rsList;
	/*
	 * Defines the locations of previously saved domain data
	 */
	public static String LOCFIRE = "../RSGridWorld/domains/fire";
	public static String LOCPIT = "../RSGridWorld/domains/pit";
	public static String LOCFIREPIT = "../RSGridWorld/domains/firepit";

	public BVRS(RewardShape domain){
		setDeterministicTransitionDynamics();
	}


	/**
	 * Will set the domain to use deterministic action transitions.
	 * [FROM BURLAP'S GRIDWORLD]
	 */
	public void setDeterministicTransitionDynamics(){
		int na = 4;
		transitionDynamics = new double[na][na];
		for(int i = 0; i < na; i++){
			for(int j = 0; j < na; j++){
				if(i != j){
					transitionDynamics[i][j] = 0.;
				}
				else{
					transitionDynamics[i][j] = 1.;
				}
			}
		}
	}

	/**
	 * Will set the movement direction probabilities based on the action chosen. The index (0,1,2,3) indicates the
	 * direction north,south,east,west, respectively and the matrix is organized by transitionDynamics[selectedDirection][actualDirection].
	 * For instance, the probability of the agent moving east when selecting north would be specified in the entry transitionDynamics[0][2]
	 * 
	 * @param transitionDynamics entries indicate the probability of movement in the given direction (second index) for the given action selected (first index).
	 * [FROM BURLAP'S GRIDWORLD]
	 */
	public void setTransitionDynamics(double [][] transitionDynamics){
		this.transitionDynamics = transitionDynamics.clone();
	}

	@Override
	public Domain generateDomain() {
		
		Domain domain = new SADomain();

		rsList = new ArrayList<RewardShape>();
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 20; j++){
				int[] ar = {i,j};
				new DomainAction(i+","+j, domain, ar);
			}
		}
		new DomainAction("-1"+","+"-1", domain, new int[] {-1, -1});

		ObjectClass domainClass = new ObjectClass(domain, CLASSDOMAIN);
		Attribute chosetermaction = new Attribute(domain, ATTCHOSETERMACTION, Attribute.AttributeType.BOOLEAN);
		domainClass.addAttribute(chosetermaction);

		new ChoseTermActionPF("ChoseTermActionPF", domain, new String[]{CLASSDOMAIN});

		//Creates a new Attribute object
		//Attribute sizeatt = new Attribute(domain, ATTSIZE, Attribute.AttributeType.INT);
		//sizeatt.setLims(0, Integer.MAX_VALUE);

		//Attribute obnumatt = new Attribute(domain, ATTOBSTACLENUM, Attribute.AttributeType.INT);
		
		//Attribute obtypeatt = new Attribute(domain, ATTOBSTACLETYPE, Attribute.AttributeType.INT);
		
		/* - These potentially used for the larger MDP
		ObjectClass domainClass = new ObjectClass(domain, CLASSDOMAIN);
		domainClass.addAttribute(sizeatt);
		domainClass.addAttribute(obnumatt);
		domainClass.addAttribute(obtypeatt);

		new DomainAction(ACTIONCHANGEGRIDSIZE, domain, this.transitionDynamics[0]);
		new DomainAction(ACTIONCHANGEOBSTACLETYPE, domain, this.transitionDynamics[1]);
		new DomainAction(ACTIONCHANGEOBSTACLETYPE, domain, this.transitionDynamics[2]);
		*/
		return domain;
	}
	public State getInitState(Domain d){
		State s = new State();

		s.addObject(new ObjectInstance(d.getObjectClass(CLASSDOMAIN), ATTCHOSETERMACTION+0));
				
		return s;
	}

	public void setTermAction(State s, boolean b){
		ObjectInstance o = s.getObjectsOfClass(CLASSDOMAIN).get(0);
		
		o.setValue(ATTCHOSETERMACTION, b);
	}

	public class DomainAction extends Action{

		/**
		 * File coordinates where the first number specifies the type (fire, pit, firepit ...) and the second the specific filename
		 */
		protected int [] coord;

		/*
		 * Designates the terminal action (solving)
		 */
		public final int[] term = {-1,-1};
		/**
		 * Initializes for the given name, domain and actually direction probabilities the agent will go
		 * @param name name of the action
		 * @param domain the domain of the action
		 * @param directions the probability for each direction (index 0,1,2,3 corresponds to north,south,east,west, respectively).
		 * @param map the map of the world
		 */
		public DomainAction(String name, Domain domain, int [] coord){
			super(name, domain, "");
			this.coord = coord;
		}
		
		@Override
		protected State performActionHelper(State st, String[] params) {
			if(Arrays.equals(this.coord, term)){
				//computeDomainPotential();
				ObjectInstance dom = st.getObjectsOfClass(CLASSDOMAIN).get(0);
				dom.setValue(ATTCHOSETERMACTION, true);
			}
			else{
				addDomainToList(this.coord);
			}
			return st;
		}
		
		@Override
		public List<TransitionProbability> getTransitions(State st, String [] params){
			return this.deterministicTransition(st, params);
		}
	}
	private void computeDomainPotential(){
		rsList.get(rsList.size()-1).experimenterAndPlotter("expDataRSRF");
	}

	//calls rewardShape with string, of which it will build a domain if the file and path exists
	private void addDomainToList(int[] coord){
		int folderNum = coord[0];
		int fileNum = coord[1];
		String fileName = fileNum + ".dat";
		String folderPath = "../RSGridWorld/domains/";
		switch(folderNum){
			case 0: folderPath += "fire/"; break;
			case 1: folderPath += "firepit/"; break;
			case 2: folderPath += "pit/"; break;
		}
		//if there are other RewardShape objects, use their states and planner for a new shaped RewardShape
		//since the nth rs is shaped by the n-1th, which is shaped by the n-2th ..., the shaping can be handled cumulatively in one step
		if(rsList.size() > 0 && rsList.size() < 3){
			RewardShape lastRS = rsList.get(rsList.size() - 1);
			RewardShape rs = new RewardShape(folderPath + fileName, .99, lastRS.getStates(), lastRS.getPlanner());
			rsList.add(rs);
		} else if(rsList.size() > 2){

		}

		else {
			RewardShape rs = new RewardShape(folderPath + fileName);
			rsList.add(rs);
		}
	}

	/**
	 * Prop function to determine if the terminating action has been executed
	 */
	public class ChoseTermActionPF extends PropositionalFunction{

		/**
		 * Initializes with given name domain and parameter object class types
		 * @param name name of function
		 * @param domain the domain of the function
		 * @param parameterClasses the object class types for the parameters
		 */
		public ChoseTermActionPF(String name, Domain domain, String[] parameterClasses) {
			super(name, domain, parameterClasses);
		}

		@Override
		public boolean isTrue(State st, String[] params) {
			
			//assuming there will only be once instance of the domain object. could change
			ObjectInstance dom = st.getObject(params[0]);
			return dom.getBooleanValForAttribute(ATTCHOSETERMACTION);
		}
	}

	public RewardShape getLastRS(){
		if(rsList.size() > 1)
			return rsList.get(rsList.size() - 1);
		else return null;
	}
	public int getRSListSize(){
		return rsList.size();
	}

}