import RSGridWorld.*;
//package RSGridWorld;
/*
 * This is the entry point into the RSGridWorld domain. It invokes the class and auxillery classes to conduct an experiment with some
 * defined algorithm.
 *
 * Author: Maxwell J Svetlik
*/
import java.awt.Color;
import java.util.List;
import java.util.*;
import java.io.PrintWriter;
import java.io.File;
import java.io.FileOutputStream;

import burlap.behavior.singleagent.*;
import burlap.domain.singleagent.gridworld.*;
import burlap.oomdp.core.*;
import burlap.oomdp.singleagent.*;
import burlap.oomdp.singleagent.common.*;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.oomdp.auxiliary.StateParser;
//visualization
import burlap.oomdp.visualizer.Visualizer;
import burlap.behavior.singleagent.EpisodeSequenceVisualizer;
//plotting visuals
import burlap.oomdp.auxiliary.StateGenerator;
import burlap.oomdp.auxiliary.common.ConstantStateGenerator;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;
//planning & learning
import burlap.behavior.singleagent.learning.*;
import burlap.behavior.singleagent.learning.tdmethods.*;
import burlap.behavior.singleagent.planning.*;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.planning.deterministic.*;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.behavior.singleagent.Policy.ActionProb;
import burlap.oomdp.core.State;
import burlap.oomdp.core.Attribute;
import burlap.oomdp.core.Attribute.AttributeType;
import burlap.oomdp.visualizer.*;
import burlap.oomdp.core.ObjectInstance;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.*;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D.PolicyGlyphRenderStyle;
import burlap.oomdp.singleagent.common.VisualActionObserver;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner.PlanningFailedException;
import burlap.behavior.singleagent.shaping.potential.PotentialShapedRF;
import burlap.behavior.singleagent.shaping.potential.PotentialFunction;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.domain.singleagent.gridworld.GridWorldTerminalFunction;
import burlap.domain.singleagent.gridworld.GridWorldTerminalFunction.IntPair;
import burlap.behavior.singleagent.planning.commonpolicies.EpsilonGreedy;
import burlap.oomdp.singleagent.explorer.VisualExplorer;
import burlap.behavior.singleagent.learning.modellearning.rmax.PotentialShapedRMax;
//graphs and plotting
import burlap.oomdp.auxiliary.StateGenerator;
import burlap.oomdp.auxiliary.common.ConstantStateGenerator;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;
import burlap.behavior.singleagent.ValueFunctionInitialization;

import burlap.behavior.learningrate.ExponentialDecayLR;

/*
* Thanks to the great conspiracy of java and burlap, 
* need a pair to return both objects to get data from source tasks
*/


public class RewardShape {

    RSGridWorld                 gwdg;
    Domain                      domain;
    StateParser                 sp;
    RewardFunction              rf;
    TerminalFunction            tf;
    StateConditionTest          goalCondition;
    State                       initialState;
    DiscreteStateHashFactory    hashingFactory;

    int                         numObstacles;
    Map<String,ArrayList<Coordinates>>     obstacleMap = new HashMap<String,ArrayList<Coordinates>>();
    Coordinates                 goalPos;
    Coordinates                 agentStartPos;
    Policy                      p;
    boolean                     term = true;
    boolean                     alpha = true;
    State                       lastState;

    public static void main(String[] args) {
        //fill with desired obstacle types. supported are: rock, fire, pit
        String [] obstacles = { RSGridWorld.OBSTACLEPIT, RSGridWorld.OBSTACLEFIRE};
        //define the number of obstacles. there will be numObstacles of each obstacle type
        int numObstacles = 5;
        String outputPath = "data/"; //directory to record results

        /*
         * VISUAL EXAMPLES
        */
        RewardShape example = new RewardShape(obstacles, numObstacles, 5, 5);
        generateDomainsToFile(obstacles);
        example = new RewardShape("domains/target/1.dat");


        //This example writes states to file, so they can be visualzied. The folling line visualizes all states
        //example.StringarsaLearningExample(outputPath, 2, 20);
        //example.visualize(outputPath);

        //This function runs many trials and plots various metrics as definied in the method
        //example.experimenterAndPlotter("expDataRF");

        //This example just outputs the no. steps till completion to the console
        //example.SarsaLearningExample(1000);

        //This example runs VIteration and displays a heatmap of the value function
        //example.ValueIterationExample(outputPath);
        //example.QLearningExample(outputPath);
        //example.visualize(outputPath);





        /*
         * RewardShaping experiments
         */
        example.findOptimalTraining();
        example.generateWeakExp(); //everything is pretty much stuffed into this
        
        /*example = new RewardShape("domains/nstep/goal4.dat", false);
        LearningPair ea = example.QPolicyEval(outputPath,0.25);
        example = new RewardShape("domains/nstep/goal6.dat", .25, ea, RSGridWorld.OBSTACLEGOAL,false);
        ea = example.QPolicyEval(outputPath,0.25);
        example = new RewardShape("domains/nstep/goal5.dat", .25, ea, RSGridWorld.OBSTACLEGOAL,false);
        ea = example.QPolicyEval(outputPath,0.25);
        */






        /*
         *  Devel area, new features, testing, etc
         */
        ArrayList<String> tasks = new ArrayList<String>(Arrays.asList("domains/nstep/goal7.dat", "domains/nstep/goal8.dat", "domains/nstep/goal9.dat",
                "domains/nstep/target5.dat", "domains/nstep/pit2.dat", "domains/nstep/pit4.dat", "domains/nstep/goal4.dat", "domains/nstep/goal5.dat",
                "domains/nstep/fire2.dat"));
        example.treeExperiments(tasks);


        /*
         * Generate Domains and visualize a specified one
         */
        //generateDomainsToFile(obstacles);
        //example.SarsaLearningExample(outputPath, 2, 50);
        //example.visualize(outputPath);

        //example = new RewardShape("domains/nstep/target2.dat");
        //example.SarsaLearningExample(outputPath, 1, 1);
        //example.visualize(outputPath);

        //example = new RewardShape("domains/nstep/firepit1.dat");
        //example.SarsaLearningExample(outputPath, 1, 1);
        //example.visualize(outputPath);

        //example = new RewardShape("domains/target/1.dat");
        //example.QLearningExample(outputPath);
        //example.visualize(outputPath);
    }

    public RewardShape(String[] obstacleNames, int numObstacles, int width, int height){
        goalPos = new Coordinates(width - 1,height - 1);
        agentStartPos = new Coordinates(0,0);
        //create the domain
        gwdg = new RSGridWorld(width, height, obstacleNames, numObstacles);
        gwdg.setDeterministicTransitionDynamics();

        domain = gwdg.generateDomain();
        //create the state parser
        sp = new RSGridWorldStateParser(domain);

        rf = new RSGridWorldRewardFunction(gwdg);
        tf = new SinglePFTF(domain.getPropFunction(RSGridWorld.PFATLOCATION));
        goalCondition = new TFGoalCondition(tf);

        //set up the initial state of the task
        initialState = RSGridWorld.getOneAgentOneLocationNObstacleState(domain, numObstacles);

        //set up the state hashing system
        hashingFactory = new DiscreteStateHashFactory();
        hashingFactory.setAttributesForClass(RSGridWorld.CLASSAGENT,
        domain.getObjectClass(RSGridWorld.CLASSAGENT).attributeList);
        RSGridWorld.setAgent(initialState, (int)agentStartPos.x, (int)agentStartPos.y);
        RSGridWorld.setLocation(initialState, 0, (int)goalPos.x, (int)goalPos.y);

        //runs a search to see if a path to the terminal goal exists
        do {placeObstacles(width, height);}
        while(!this.BFS());

        tf = new RSGridWorldTerminalFunction(initialState); //generateTerminalCoordinates(initialState);

    }
    public RewardShape(String[] obstacleNames, int numObstacles, int width, int height, String domainOutFile){
        this(obstacleNames, numObstacles, width, height);
        DomainToFile dtf = new DomainToFile();
        dtf.toFile(domainOutFile, gwdg, initialState);
    }
    public RewardShape(String domainInFile, boolean neverEndTF){
        DomainToFile dtf = new DomainToFile();
        dtf.parseFile(domainInFile);
        gwdg = dtf.getDomain();
        initialState = dtf.getState();
        lastState = initialState;
        gwdg.setDeterministicTransitionDynamics();

        domain = gwdg.generateDomain();
        //create the state parser
        sp = new RSGridWorldStateParser(domain);

        p = new BiasedPolicy(.1);

        if(neverEndTF){
            rf = new RSGridWorldRewardFunction(false,gwdg);
        }
        else
            rf = new RSGridWorldRewardFunction(gwdg);


        //set up the state hashing system
        hashingFactory = new DiscreteStateHashFactory();
        hashingFactory.setAttributesForClass(RSGridWorld.CLASSAGENT,domain.getObjectClass(RSGridWorld.CLASSAGENT).attributeList);
        if(neverEndTF){
            //initialState = RSGridWorld.getOneAgentNObstacleState(domain, numObstacles);
            //tf = new SinglePFTF(domain.getPropFunction(RSGridWorld.PFNEVERTERM));
            tf = new RSGridWorldTerminalFunction(initialState,false);

        }else{
            tf = new RSGridWorldTerminalFunction(initialState);
            goalCondition = new TFGoalCondition(tf);
        }
    }
    public RewardShape(String domainInFile){
        this(domainInFile, false);
    
}
    /*
     * Constructor for incrementally loading previous states into the next domain
     */
    public RewardShape(String domainInFile, double rs_discount, LearningPair ea, String featureName, boolean neverEndTF){
        this(domainInFile, neverEndTF);
        RSGridWorldAgentSpaceBuilder asb = new RSGridWorldAgentSpaceBuilder(ea, featureName);
        HashMap<HashSet<Coordinates>, Double> agentValueMap = asb.getASTuple();
        ArrayList<HashMap<HashSet<Coordinates>,Double> > asbList = new ArrayList<HashMap<HashSet<Coordinates>,Double> >();
        asbList.add(agentValueMap);
        p = new BiasedPolicy(0.1, asbList, featureName);
        ((BiasedPolicy )p).setPlanner((OOMDPPlanner)ea.la);
        if(RSGridWorld.DEBUG)
            printSourceHeatMap(asbList);
        RSGridWorldPotentialFunction rsgwpf = new RSGridWorldPotentialFunction(agentValueMap, featureName, p);
        PotentialFunction[] temp = {rsgwpf};
        rf = new RSPotentialShapedRF(new RSGridWorldRewardFunction(gwdg), temp, rs_discount);
    }
    /*
     * This allows the usage of a potential function that doesn't use forward advice.
     * Specifically, if stateAction = false, the potential function thats used uses distance only on states.
     */
    public RewardShape(String domainInFile, double rs_discount, LearningPair ea, String featureName, boolean neverEndTF, boolean stateAction){
        this(domainInFile, neverEndTF);
        RSGridWorldAgentSpaceBuilder asb = new RSGridWorldAgentSpaceBuilder(ea, featureName);
        HashMap<HashSet<Coordinates>, Double> agentValueMap = asb.getASTuple();
        ArrayList<HashMap<HashSet<Coordinates>,Double> > asbList = new ArrayList<HashMap<HashSet<Coordinates>,Double> >();
        asbList.add(agentValueMap);
        p = new BiasedPolicy(0.1, asbList, featureName);
        ((BiasedPolicy )p).setPlanner((OOMDPPlanner)ea.la);
        if(RSGridWorld.DEBUG)
            printSourceHeatMap(asbList);
        RSGridWorldPotentialFunction rsgwpf = new RSGridWorldPotentialFunction(agentValueMap, featureName, p);
        rsgwpf.setStateAction(stateAction);
        rsgwpf.setPlanner((QComputablePlanner)ea.la);
        PotentialFunction[] temp = {rsgwpf};
        rf = new RSPotentialShapedRF(new RSGridWorldRewardFunction(gwdg), temp, rs_discount);
    }
    public RewardShape(String domainInFile, double rs_discount, LearningPair ea, LearningPair eaprime, String featureName, boolean neverEndTF){
        this(domainInFile, neverEndTF);
        RSGridWorldAgentSpaceBuilder asb = new RSGridWorldAgentSpaceBuilder(ea, featureName);
        HashMap<HashSet<Coordinates>, Double> agentValueMap = asb.getASTuple();
        RSGridWorldAgentSpaceBuilder asbprime = new RSGridWorldAgentSpaceBuilder(eaprime, featureName);
        HashMap<HashSet<Coordinates>, Double> agentValueMapPrime = asbprime.getASTuple();
        ArrayList<HashMap<HashSet<Coordinates>,Double> > asbList = new ArrayList<HashMap<HashSet<Coordinates>,Double> >();
        for(HashSet<Coordinates> t : agentValueMap.keySet()){
            double sum = 0.0;
            if(agentValueMapPrime.containsKey(t)){
                sum += agentValueMapPrime.get(t);
            }
            sum+= agentValueMap.get(t);
            agentValueMapPrime.put(t, sum);
        }
        asbList.add(agentValueMapPrime);
        p = new BiasedPolicy(0.1, asbList, featureName);
        ((BiasedPolicy )p).setPlanner((OOMDPPlanner)ea.la);
        RSGridWorldPotentialFunction rsgwpf = new RSGridWorldPotentialFunction(agentValueMapPrime, featureName, p);
        PotentialFunction[] temp = {rsgwpf};
        rf = new RSPotentialShapedRF(new RSGridWorldRewardFunction(gwdg), temp, rs_discount);
    }
    /*
     * N-step transfer
     */
    public RewardShape(String domainInFile, double rs_discount, LearningPair[] ea, String[] featureName, boolean neverEndTF){
        this(domainInFile, neverEndTF);
        PotentialFunction temp[] = new PotentialFunction[featureName.length];
        ArrayList<HashMap<HashSet<Coordinates>,Double> > asbList = new ArrayList<HashMap<HashSet<Coordinates>,Double> >();
        for(int i = 0; i < ea.length; i++){
            ArrayList<HashMap<HashSet<Coordinates>,Double> > tempasbList = new ArrayList<HashMap<HashSet<Coordinates>,Double> >();
            RSGridWorldAgentSpaceBuilder asb = new RSGridWorldAgentSpaceBuilder(ea[i], featureName[i]);
            HashMap<HashSet<Coordinates>, Double> agentValueMap = asb.getASTuple();
            asbList.add(agentValueMap);
            tempasbList.add(agentValueMap);
            Policy pi = new BiasedPolicy(0.1, tempasbList, featureName);
            ((BiasedPolicy )pi).setPlanner((OOMDPPlanner)ea[i].la);
            RSGridWorldPotentialFunction rsgwpf = new RSGridWorldPotentialFunction(agentValueMap, featureName[i], pi);
            temp[i] = rsgwpf;
        }
        p = new BiasedPolicy(0.1,asbList,RSGridWorld.OBSTACLEGOAL);

        for(int i = 0; i < ea.length; i++){
            ((RSGridWorldPotentialFunction)(temp[i])).setPolicy(p);
        }
        rf = new RSPotentialShapedRF(new RSGridWorldRewardFunction(gwdg), temp, rs_discount);
    }

    public RewardShape(String domainInFile, double rs_discount, LearningPair[] ea, String[] featureName, boolean neverEndTF, boolean[] stateaction){
        this(domainInFile, neverEndTF);
        PotentialFunction temp[] = new PotentialFunction[featureName.length];
        ArrayList<HashMap<HashSet<Coordinates>,Double> > asbList = new ArrayList<HashMap<HashSet<Coordinates>,Double> >();
        for(int i = 0; i < ea.length; i++){
            if(stateaction[i]){
                ArrayList<HashMap<HashSet<Coordinates>,Double> > tempasbList = new ArrayList<HashMap<HashSet<Coordinates>,Double> >();
                RSGridWorldAgentSpaceBuilder asb = new RSGridWorldAgentSpaceBuilder(ea[i], featureName[i]);
                HashMap<HashSet<Coordinates>, Double> agentValueMap = asb.getASTuple();
                asbList.add(agentValueMap);
                tempasbList.add(agentValueMap);
                Policy pi = new BiasedPolicy(0.1, tempasbList, featureName);
                ((BiasedPolicy )pi).setPlanner((OOMDPPlanner)ea[i].la);
                RSGridWorldPotentialFunction rsgwpf = new RSGridWorldPotentialFunction(new HashMap<HashSet<Coordinates>, Double>(agentValueMap), featureName[i], pi);
                temp[i] = rsgwpf;
            }
            else{
                ArrayList<HashMap<HashSet<Coordinates>,Double> > tempasbList = new ArrayList<HashMap<HashSet<Coordinates>,Double> >();
                RSGridWorldAgentSpaceBuilder asb = new RSGridWorldAgentSpaceBuilder(ea[i], featureName[i], stateaction[i]);
                HashMap<HashSet<Coordinates>, Double> agentValueMap = asb.getASTuple();
                RSGridWorldPotentialFunction rsgwpf = new RSGridWorldPotentialFunction(agentValueMap, featureName[i], p);
                rsgwpf.setStateAction(false);
                rsgwpf.setPlanner((QComputablePlanner)ea[i].la);
                temp[i] = rsgwpf;
            }
        }
        p = new BiasedPolicy(0.1,asbList,RSGridWorld.OBSTACLEGOAL);
        if(RSGridWorld.DEBUG)
            printSourceHeatMap(asbList);

        for(int i = 0; i < ea.length; i++){
            ((RSGridWorldPotentialFunction)(temp[i])).setPolicy(p);
        }
        rf = new RSPotentialShapedRF(new RSGridWorldRewardFunction(gwdg), temp, rs_discount);
    }

    public RewardShape(String domainInFile, double rs_discount, LearningPair eaprime, LearningPair[] ea, String[] featureName, boolean neverEndTF){
        this(domainInFile, neverEndTF);
        PotentialFunction temp[] = new PotentialFunction[featureName.length];
        ArrayList<HashMap<HashSet<Coordinates>,Double> > asbList = new ArrayList<HashMap<HashSet<Coordinates>,Double> >();
        for(int i = 0; i < ea.length; i++){
            ArrayList<HashMap<HashSet<Coordinates>,Double> > tempasbList = new ArrayList<HashMap<HashSet<Coordinates>,Double> >();
            RSGridWorldAgentSpaceBuilder asb = new RSGridWorldAgentSpaceBuilder(ea[i], featureName[i]);
            HashMap<HashSet<Coordinates>, Double> agentValueMap = asb.getASTuple();
            if(i == ea.length -1){
                RSGridWorldAgentSpaceBuilder asbprime = new RSGridWorldAgentSpaceBuilder(eaprime, featureName[i-1]);
                HashMap<HashSet<Coordinates>, Double> agentValueMapPrime = asbprime.getASTuple();
                for(HashSet<Coordinates> t : agentValueMap.keySet()){
                    double sum = 0.0;
                    if(agentValueMapPrime.containsKey(t)){
                        sum += agentValueMapPrime.get(t);
                    }
                    sum+= agentValueMap.get(t);
                    agentValueMapPrime.put(t, sum);
                }
            }
            asbList.add(agentValueMap);
            tempasbList.add(agentValueMap);
            Policy pi = new BiasedPolicy(0.1, tempasbList, featureName);
            ((BiasedPolicy )pi).setPlanner((OOMDPPlanner)ea[i].la);
            RSGridWorldPotentialFunction rsgwpf = new RSGridWorldPotentialFunction(agentValueMap, featureName[i], pi);
            temp[i] = rsgwpf;
        }
        p = new BiasedPolicy(0.1,asbList,RSGridWorld.OBSTACLEGOAL);
        if(RSGridWorld.DEBUG)
            printSourceHeatMap(asbList);

        for(int i = 0; i < ea.length; i++){
            ((RSGridWorldPotentialFunction)(temp[i])).setPolicy(p);
        }
        rf = new RSPotentialShapedRF(new RSGridWorldRewardFunction(gwdg), temp, rs_discount);
    }
    public void generateWeakExp(){
        smallExperiments();
        //largeExperiments();
    }

    public ArrayList<String> getTreeList(TreeNode<String> root){
        ArrayList<String> nodes = new ArrayList<String>();
        getChildren(root, nodes);
        System.out.println(nodes.toString());
        return nodes;
    }

    private void getChildren(TreeNode<String> node, ArrayList<String> list){
        List<TreeNode<String>> children = node.getChildren();
        for(TreeNode<String> child : children)
            getChildren(child, list);
        list.add(node.getData());
    }

    public void treeExperiments(ArrayList<String> tasks){
        String outputPath = "data/";
        TreeNode<String> root =  this.getTransferTree(tasks, "domains/nstep/target5.dat");
        ArrayList<String> nodes = getTreeList(root);
        int trials = 100;
        RewardShape example = new RewardShape("domains/nstep/pit3.dat", false);
        LearningPair ea;
        double gamma = 1.;
        int n = 3;
        String[] obname = new String[n];
        LearningPair[] ea_ar = new LearningPair[n];
        boolean[] stateaction = new boolean[n];

/*
            example = new RewardShape("domains/nstep/goal7.dat", false);
            ea = example.QPolicyEval(outputPath,5.,gamma);

            example = new RewardShape("domains/nstep/goal8.dat", gamma, ea, "goal",false,false);
            ea = example.QPolicyEval(outputPath,5.,gamma);
            example = new RewardShape("domains/nstep/goal9.dat", gamma, ea, "goal",false,false);
            ea = example.QPolicyEval(outputPath,10.,gamma);
*/
       // findOptimalTraining();

        /*for(int j = 0; j < trials; j++){
            gamma=0.7;
            example = new RewardShape("domains/nstep/pit4.dat", true);
            ea = example.QPolicyEval(outputPath,200,gamma);
            obname[0] = RSGridWorld.OBSTACLEPIT;
            ea_ar[0] = ea;
            stateaction[0] = true;

            example = new RewardShape("domains/nstep/fire2.dat", true);
            ea = example.QPolicyEval(outputPath,400,gamma);
            obname[1] = RSGridWorld.OBSTACLEFIRE;
            ea_ar[1] = ea;
            stateaction[1] = true;


            example = new RewardShape("domains/nstep/goal7.dat", false);
            ea = example.QPolicyEval(outputPath,300,gamma);
            obname[2] = RSGridWorld.OBSTACLEGOAL;
            ea_ar[2] = ea;
            stateaction[2] = false;


            example = new RewardShape("domains/nstep/goal8.dat", gamma, ea_ar[2], obname[2],false,stateaction[2]);
            ea = example.QPolicyEval(outputPath,400,gamma);
            obname[2] = RSGridWorld.OBSTACLEGOAL;
            ea_ar[2] = ea;
            stateaction[2] = false;


            example = new RewardShape("domains/nstep/goal9.dat", gamma, ea_ar[2], obname[2],false,stateaction[2]);
            ea = example.QPolicyEval(outputPath,700,gamma);
            obname[2] = RSGridWorld.OBSTACLEGOAL;
            ea_ar[2] = ea;
            stateaction[2] = false;

            example = new RewardShape("domains/nstep/target5.dat", gamma, ea_ar, obname, false, stateaction);
            example.experimenterAndPlotter(outputPath+"target5_tree_p4_f2_g789_2000_rwd200_"+gamma,true,j);
        }*/
        
       
    }
    public void largeExperiments(){
        double discount = 0.99;
        String outputPath = "data/";
        RewardShape example = new RewardShape("domains/nstep/pit3.dat", false);
        LearningPair ea;
        double gamma = 0.25;
        int trials = 10;
        /*
         * Baseline
         */
        if(true)
            for(int j = 0; j < trials; j++){
                example = new RewardShape("domains/target/1.dat", false);
                example.experimenterAndPlotter(outputPath+"target1_baseline",false,j);
            }
        //example = new RewardShape("domains/target/1.dat", false);
        //example.experimenterAndPlotter(outputPath+"target1_baseline",false);

    }
    public void smallExperiments(){
        /*
         * One step transfers
         */
        double discount = 0.99;
        String outputPath = "data/";
        RewardShape example = new RewardShape("domains/nstep/pit3.dat", false);
        //example.experimenterAndPlotter("baseline");
        LearningPair ea;
        //example = new RewardShape("domains/nstep/pit3.dat",true);
        //example.QPolicyEval(outputPath, 1000);
        //example.visualize(outputPath);
        //double gamma[] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
        //double gamma[] = {.25};
        double gamma = 0.25;
        int trials = 10;
        /*
         * Baseline
         */
        if(false)
            for(int j = 0; j < trials; j++){
                example = new RewardShape("domains/nstep/target5.dat", false);
                example.experimenterAndPlotter(outputPath+"target5_baseline",false,j);
            }
        //example = new RewardShape("domains/target/1.dat", false);
        //example.experimenterAndPlotter(outputPath+"target1_baseline",false);

        /*
        * Pit
        */

        if(false){
        //get baseline
        
        //learn with forward advice
        	for(int j = 0; j < trials; j++){
                gamma=0.25;
                term = false;
		        example = new RewardShape("domains/nstep/pit2.dat",true);
		        ea = example.QPolicyEval(outputPath, 90000, gamma);
		        term = true;
                gamma=.25;
                example = new RewardShape("domains/nstep/target5.dat", gamma, ea, RSGridWorld.OBSTACLEPIT, false);
		        example.experimenterAndPlotter(outputPath+"pit2_to_target5_gamma"+gamma, true,j);
		    }
        }

        /*
        * Fire
        */

        if(false){

            for(int j = 0; j < trials; j++){
                gamma=0.99;
                example = new RewardShape("domains/nstep/fire2.dat",true);
                ea = example.QPolicyEval(outputPath,800,gamma);
                gamma=.99;
                example = new RewardShape("domains/nstep/target5.dat", gamma, ea, RSGridWorld.OBSTACLEFIRE, false);
                example.experimenterAndPlotter(outputPath+"fire2_to_target5_gamma"+gamma,true,j);
            }
        }

        /*
        * Music
        */

        if(false){
            example = new RewardShape("domains/nstep/music2.dat", true);
            //example.experimenterAndPlotter("music5_baseline",false);
            
            for(int j = 0; j < trials; j++){
                example = new RewardShape("domains/nstep/music2.dat",true);
                ea = example.QPolicyEval(outputPath,10000, gamma);
                example = new RewardShape("domains/nstep/target6.dat", gamma, ea, RSGridWorld.OBSTACLEMUSIC, false);
                example.experimenterAndPlotter(outputPath+"music2_to_target6_gamma"+gamma,true,j);
            }
        }

        /*
         * Goal
         */
        if(false){
            example = new RewardShape("domains/nstep/goal5.dat", false);
            gamma = 0.7;
            for(int j = 0; j < trials; j++){
                term=true;
                example = new RewardShape("domains/nstep/goal5.dat",false);
                ea = example.QPolicyEval(outputPath,24600, gamma);
                term = true;
                example = new RewardShape("domains/nstep/target5.dat", gamma, ea, RSGridWorld.OBSTACLEGOAL, false);
                example.experimenterAndPlotter(outputPath+"goal5_to_target5_gamma"+gamma,true,j);
            }
        }

        /*
         * n step transfers
         */
        if(true){
            int n = 4;
            List<List<State>> previousStates = new ArrayList<List<State>>();
            OOMDPPlanner[] previousPlanner = new OOMDPPlanner[n];
            String[] obname = new String[n];
            LearningPair[] ea_ar = new LearningPair[n];
            boolean[] stateaction = new boolean[n];
            /*
             * baseline
             */
            if(false){
                example = new RewardShape("domains/nstep/target5.dat", false);
                example.experimenterAndPlotter(outputPath+"target5_2s_baseline",false,0);
            }
            /*
            * Learn 3 potential functions, for each obstacle (and a goal)
            */
            if(true){
                for(int j = 0; j < trials; j++){
                    gamma=0.7;
                    example = new RewardShape("domains/nstep/fire2.dat", true);
                    ea = example.QPolicyEval(outputPath,400,gamma);
                    obname[0] = RSGridWorld.OBSTACLEFIRE;
                    ea_ar[0] = ea;
                    stateaction[0] = true;

                    example = new RewardShape("domains/nstep/pit2.dat", true);
                    ea = example.QPolicyEval(outputPath,400,gamma);
                    obname[1] = RSGridWorld.OBSTACLEPIT;
                    ea_ar[1] = ea;
                    stateaction[1] = true;
                    term = false;
                    gamma=0.7;

                    example = new RewardShape("domains/nstep/goal9.dat", false);
                    ea = example.QPolicyEval(outputPath,2000,0.7);
                    obname[2] = RSGridWorld.OBSTACLEGOAL;
                    ea_ar[2] = ea;
                    stateaction[2] = false;
                    term=true;

                    example = new RewardShape("domains/nstep/goal4.dat", gamma, ea_ar[2], obname[2],false,stateaction[2]);
                    ea = example.QPolicyEval(outputPath,1500,0.7);
                    obname[3] = RSGridWorld.OBSTACLEGOAL;
                    ea_ar[3] = ea;
                    stateaction[3] = true;

                    example = new RewardShape("domains/nstep/target5.dat", gamma, ea_ar, obname, false, stateaction);
                    example.experimenterAndPlotter(outputPath+"target5_fire2_pit2_goal9_into_goal4"+gamma,true,j);
                }
            }
            /*
            * curriculum
            */ 
            findOptimalTraining();
            /*if(false){
                gamma = 0.99;
                for(int j = 0; j < trials; j++){
                    gamma = 0.7;
                    //alpha = false;
                    example = new RewardShape("domains/nstep/fire2.dat", true);
                    ea = example.QPolicyEval(outputPath,800,gamma);
                    obname[0] = RSGridWorld.OBSTACLEFIRE;
                    ea_ar[0] = ea;

                    example = new RewardShape("domains/nstep/pit2.dat", true);
                    ea = example.QPolicyEval(outputPath,800,gamma);
                    obname[1] = RSGridWorld.OBSTACLEPIT;
                    ea_ar[1] = ea;
                    term = true;
                    //alpha = true;
                    gamma = 0.7;

                    example = new RewardShape("domains/nstep/goal4.dat", false);
                    ea = example.QPolicyEval(outputPath,800,gamma);
                    System.out.println("Number of steps to convergence in goal4:" + ((EpisodeAnalysis)ea.ep).numTimeSteps());

                    example = new RewardShape("domains/nstep/goal6.dat", gamma, ea, RSGridWorld.OBSTACLEGOAL,false);
                    LearningPair ea2 = example.QPolicyEval(outputPath,1200,gamma);
                    System.out.println("Number of steps to convergence in goal6:" + ((EpisodeAnalysis)ea.ep).numTimeSteps());

                    example = new RewardShape("domains/nstep/goal5.dat", gamma,ea2,ea, RSGridWorld.OBSTACLEGOAL,false);
                    ea = example.QPolicyEval(outputPath,4000,gamma);
                    obname[2] = RSGridWorld.OBSTACLEGOAL;
                    ea_ar[2] = ea;
                    System.out.println("Number of steps to convergence in goal5:" + ((EpisodeAnalysis)ea.ep).numTimeSteps());
                    //PotentialShapedRMax psrmax = new PotentialShapedRMax(domain, rf, tf, .7, hashingFactory, 100, 10, .05, 100);
                    term = true;
                    example = new RewardShape("domains/nstep/target5.dat", gamma,ea,ea_ar, obname, false);
                    example.experimenterAndPlotter(outputPath+"target5_curriculum3strong"+gamma,true,j);
                }
            }

            /*
            * Learn 1 potential function, chaining them into subsequently larger state spaces. goal transfer only
            */

            /*if(false){
                gamma = 0.7;
                for(int j = 0; j < trials; j++){
                    term = true;
                    example = new RewardShape("domains/nstep/goal4.dat", false);
                    ea = example.QPolicyEval(outputPath,1600,gamma);
                    obname[0] = RSGridWorld.OBSTACLEGOAL;
                    ea_ar[0] = ea;
                    System.out.println("Number of steps to convergence in goal4:" + ((EpisodeAnalysis)ea.ep).numTimeSteps());

                    example = new RewardShape("domains/nstep/goal6.dat", gamma, ea, RSGridWorld.OBSTACLEGOAL,false);
                    ea = example.QPolicyEval(outputPath,4000,gamma);
                    obname[1] = RSGridWorld.OBSTACLEGOAL;
                    ea_ar[1] = ea;
                    System.out.println("Number of steps to convergence in goal6:" + ((EpisodeAnalysis)ea.ep).numTimeSteps());

                    example = new RewardShape("domains/nstep/goal5.dat", gamma, ea, RSGridWorld.OBSTACLEGOAL,false);
                    ea = example.QPolicyEval(outputPath,16000,gamma);
                    obname[2] = RSGridWorld.OBSTACLEGOAL;
                    ea_ar[2] = ea;
                    System.out.println("Number of steps to convergence in goal5:" + ((EpisodeAnalysis)ea.ep).numTimeSteps());
                    //PotentialShapedRMax psrmax = new PotentialShapedRMax(domain, rf, tf, .7, hashingFactory, 100, 10, .05, 100);
                    term = true;
                    example = new RewardShape("domains/nstep/target5.dat", gamma, ea, RSGridWorld.OBSTACLEGOAL, false);
                    example.experimenterAndPlotter(outputPath+"target5_goal4_goal6_goal5_averagestrong"+gamma,true,j);
                }
            }*/
        }
    }

    /*
    * This method incrementally increases training in source tasks, comparing the number actions used in
    * training and the absolute gain of area under the curve, compared to a target task.
    */

    public void findOptimalTraining(){
        String fp_baseline = "data/target5_baseline.csv";
        String fp_goal1    = "data/goal4_train";
        String fp_goal2    = "data/goal6_train";
        String fp_goal3    = "data/goal5_train";
        double gamma_train = 0.7;
        double gamma_eval = 0.9;
        double epsilon = 0.1;
        LearningAgent agent = new QLearning(domain, rf, tf, gamma_train, hashingFactory, 0, 0.1);
        LearningPair lp = new LearningPair();
        lp.setAgent(agent);
        int max_training = 1500;
        int step = 10;
        PrintWriter writer = null;
        int trials = 10;
        RewardShape example;


        DataAnalysis da_baseline = new DataAnalysis(fp_baseline,0);
        da_baseline.parseData();
        double baseline_area = da_baseline.getArea();

        runThreadedExperimenter(fp_baseline, "data/training/target5_goal6.csv", "domains/nstep/" , "goal6.dat", "domains/nstep/", "target5.dat");
        /*try{
            writer = new PrintWriter(new FileOutputStream(new File("data/training/goal5_training.csv"), true)); //append 
            writer.println("Trial,"+"Training Steps,"+"Area Gain (Source - Target),");
        }
        catch(Exception e){
            e.printStackTrace();
        } finally {

            for(int k = 0; k < trials; k++){
                agent = new QLearning(domain, rf, tf, gamma_train, hashingFactory, 0, 0.1);
                lp = new LearningPair();
                lp.setAgent(agent);
                for(int i = 0; i < max_training; i+=step){
                    example = new RewardShape("domains/nstep/goal5.dat", false);
                    lp = example.QPolicyEval(lp, step,epsilon);

                    /*
                    * evaluation
                    */
/*                    int j = 0;
                    example = new RewardShape("domains/nstep/target5.dat", gamma_eval, lp, RSGridWorld.OBSTACLEGOAL, false);
                    example.experimenterAndPlotter("data/training/target5_goal5"+gamma_train,true,j);
                    DataAnalysis training = new DataAnalysis("data/training/target5_goal5"+gamma_train+".csv",i);
                    training.parseData();
                    double training_area = training.getArea();
                    writer.println(k+","+i+","+(training_area - baseline_area)+",");
                    writer.flush();
                }
            }
            writer.close();
        }*/
    }

    public TreeNode<String> getTransferTree(ArrayList<String> taskFileList, String targetTask){
        //Map from groups to taskFiles
        //HashMap<HashSet<String>, HashSet<String> > taskGroups = new HashMap<HashSet<String>, HashSet<String> >();
        HashMap<ArrayList<String>, List<Tuple<String, Double, Double> > > taskGroups = new HashMap<ArrayList<String>, List<Tuple<String, Double, Double> > >();


        /*
         * Step 1, group tasks based on features.
         */
        for(String taskFile : taskFileList){
            CompareTask ctTarget = new CompareTask(taskFile, targetTask);
            double rawVal;
            if(ctTarget.isSourceCoarse())
                rawVal = ctTarget.getApplicability() / (ctTarget.getTargetSpace() - ctTarget.getSourceSpace());
            else
                rawVal = ctTarget.getApplicability() / (ctTarget.getTargetSpace() - ctTarget.getSourceSpace());
            System.out.println("Source: " + taskFile + " tp: " + rawVal);
            if(rawVal > 0.8 && rawVal != Double.POSITIVE_INFINITY){
                Tuple<String, Double, Double> tup = new Tuple<String, Double, Double>(taskFile, rawVal, ctTarget.getSourceSpace());
            
                CompareTask ct = new CompareTask();
                ArrayList<String> taskFeatures = new ArrayList<String>(ct.getTaskFeatures(taskFile));
                if(!taskGroups.containsKey(taskFeatures)){
                    ArrayList<Tuple<String, Double, Double>> tupSet = new ArrayList<Tuple<String, Double, Double>>();
                    tupSet.add(tup);
                    taskGroups.put(taskFeatures, tupSet);
                }
                else{
                    List<Tuple<String, Double, Double>> tasks = taskGroups.get(taskFeatures);
                    tasks.add(tup);
                    taskGroups.put(taskFeatures, tasks);
                }
            }
        }
        

        /*
         * Sort based on transfer value
         */
        for(ArrayList<String> group : taskGroups.keySet()){
            List<Tuple<String, Double, Double> > tasks = taskGroups.get(group);

            Comparator<Tuple<String,Double, Double>> comparator = new Comparator<Tuple<String, Double,Double>>(){

                public int compare(Tuple<String, Double, Double> tupleA, Tuple<String, Double,Double> tupleB){
                    return tupleA.getY().compareTo(tupleB.getY());
                }
            };
            System.out.println(tasks.toString());
            Collections.sort(tasks,comparator);
            System.out.println(tasks.toString());
        }

        /*
         * Now, with the groups separated and sorted, build the curriculum chains
         */
        TreeNode<String> root = new TreeNode<String>(targetTask);
        for(ArrayList<String> group : taskGroups.keySet()){
            ArrayList<Tuple<String, Double, Double> > tasks = new ArrayList<Tuple<String,Double,Double> >(taskGroups.get(group));
            int bumper = 0;
            TreeNode<String> child = new TreeNode<String>(tasks.get(0).getX());
            for(int i = 0; i < tasks.size() - 1; i++){
                double maxVal = Double.MIN_VALUE;
                String maxTask = "";
                for(int j = 0; j < tasks.size(); j++){
                    CompareTask ct_source = new CompareTask(tasks.get(i).getX(), tasks.get(j).getX());
                    //CompareTask ct_target = new CompareTask(tasks.get(i).getX(), targetTask);
                    System.out.println("Comparing: " + tasks.get(i).getX() + " and " + tasks.get(j).getX());
                    double tf = 1.0;
                    double targetState = ct_source.getTargetSpace();
                    double sourceState = ct_source.getSourceSpace();

                    /*
                     * If it is a 'coarse grained' task, account for its additional appl
                     */
                    /*if(ct.isSourceCoarse())
                        sourceState *= sourceState;
                    if(ct.isTargetCoarse())
                        targetState *= targetState;
                    */
                    if(ct_source.isSourceCoarse() && !ct_source.isTargetCoarse())
                        sourceState *= sourceState;
                    
                    tf = ct_source.getApplicability() / (targetState - sourceState);
                    if(tf == Double.POSITIVE_INFINITY)
                        tf = 0; 
                    System.out.println("Appl: " + ct_source.getApplicability());
                    System.out.println("Stspc: " + (ct_source.getTargetSpace() - (ct_source.getSourceSpace())));
                    System.out.println("Tf score: " + tf);

                    if(tf > 0 && tf > maxVal){
                        maxVal = tf;
                        maxTask = tasks.get(j).getX();
                        bumper = j;
                    }
                }
                bumper++;
                if(maxTask != ""){
                    TreeNode<String> parent = new TreeNode<String>(maxTask);
                    parent.addChild(child);
                    child = parent;
                    System.out.println("Max task: " + maxTask);
                    maxTask = "";
                }
            }
            root.addChild(child);
        }
        root.printTree();
        
        return root;
    }



    public void printTargetHeatMap(EpisodeAnalysis ea){
        int[][] heatmap = new int[10][10];
        String xa = RSGridWorld.ATTX;
        String ya = RSGridWorld.ATTY;
        
        for(int step = 1; step < ea.maxTimeStep(); step++){
            State s = ea.getState(step);
            ObjectInstance a = s.getObjectsOfClass(RSGridWorld.CLASSAGENT).get(0);
            int ax = a.getIntValForAttribute(xa);
            int ay = a.getIntValForAttribute(ya);
            heatmap[ax][ay]++;
        }
        for(int y = 9; y >= 0; y--){
            for(int x = 0; x < 10; x++)
                System.out.print(heatmap[x][y] + ",");
            System.out.print("\n");
        }
    }

    public void printSourceHeatMap(ArrayList<HashMap<HashSet<Coordinates>, Double>> avm){
        
        String actname[] = {RSGridWorld.ACTIONNORTH,RSGridWorld.ACTIONSOUTH,RSGridWorld.ACTIONEAST,RSGridWorld.ACTIONWEST};
        for(int k = 0; k < avm.size(); k++){
            HashMap<HashSet<Coordinates>, Double> tuple = avm.get(k);
            for(int y = 9; y >= 0; y--){
                for(int x =0;x < 10; x++){
                    HashSet<Coordinates> t = new HashSet<Coordinates>();
                    Coordinates maxTup = null;
                    double maxVal = Integer.MIN_VALUE;
                    for(int i = 0; i < 4; i++){
                        Coordinates testTuple = new Coordinates(-x,-y, actname[i], RSGridWorld.OBSTACLEGOAL);
                        t.clear();
                        t.add(testTuple);
                        if(tuple.containsKey(t) && tuple.get(t) > maxVal){
                            maxTup = testTuple;
                            maxVal = tuple.get(t);
                        }
                    }
                    if(maxTup == null)
                        System.out.print("" + "null" + ","+"null"+",");
                    else
                        System.out.print("" + maxVal + ","+maxTup.actionName+",");
                }
                System.out.print("\n");
            }
        }
    }
    /*
    * allows one to control the agent and walk through the domain with the key presses
    */
    public void walker(){
        Visualizer v = RSGridWorldVisualizer.getVisualizer(gwdg.getMap());
        VisualExplorer exp = new VisualExplorer(domain, v, initialState);
        
        exp.addKeyAction("w", "north");
        exp.addKeyAction("s", "south");
        exp.addKeyAction("d", "east");
        exp.addKeyAction("a", "west");
    
        exp.initGUI();
    }

    public GridWorldTerminalFunction generateTerminalCoordinates(State s){
        GridWorldTerminalFunction tf = new GridWorldTerminalFunction();
        List<ObjectInstance> loc = s.getObjectsOfClass(RSGridWorld.CLASSLOCATION);
        tf.markAsTerminalPosition(loc.get(0).getIntValForAttribute(RSGridWorld.ATTX), loc.get(0).getIntValForAttribute(RSGridWorld.ATTY));

        List<ObjectInstance> obst = s.getObjectsOfClass(RSGridWorld.CLASSOBSTACLE);
        for(ObjectInstance l : obst){
            if(l.getStringValForAttribute(RSGridWorld.ATTNAME).equals(RSGridWorld.OBSTACLEPIT))
               tf.markAsTerminalPosition(l.getIntValForAttribute(RSGridWorld.ATTX), l.getIntValForAttribute(RSGridWorld.ATTY));
        }
        return tf;
    }

    //generates obstacles based on constant definitions.
    public void placeObstacles(int width, int height){
        ArrayList<Coordinates> obstacleCoord = new ArrayList<Coordinates>();
        Random rand = new Random();
        boolean cellOccupied = true;
        //add goal and agent as occupied cells
        obstacleCoord.add(goalPos);
        obstacleCoord.add(agentStartPos);
        for(int j = 0; j < RSGridWorld.OBSTACLECLASSNAMES.length; j++){
            for(int i = 0; i < RSGridWorld.NUMOBSTACLES; i++){
                while(cellOccupied){
                    int randomX = rand.nextInt(width);
                    int randomY = rand.nextInt(height);
                    Coordinates temp = new Coordinates(randomX, randomY);
                    if(RSGridWorld.DEBUG)
                        System.out.println("X: " + randomX + " Y: " + randomY);
                    cellOccupied = obstacleCoord.contains(temp);
                    if(!cellOccupied)
                        obstacleCoord.add(temp);
                }
                int x = (int)obstacleCoord.get(obstacleCoord.size() - 1).x;
                int y = (int)obstacleCoord.get(obstacleCoord.size() - 1).y;
                gwdg.setObstacle(initialState, i + (RSGridWorld.NUMOBSTACLES*j), x ,y, RSGridWorld.OBSTACLECLASSNAMES[j]);
                if(RSGridWorld.DEBUG)
                    System.out.println("Placed obstacle at " + obstacleCoord.get(obstacleCoord.size() - 1).x + ", " + obstacleCoord.get(obstacleCoord.size() - 1).y);
                cellOccupied = true;
            }
        }
    }

    //This Breadth first search is used to ensure that a path exists for a generated RSGridWorld.
    //Therefore it has been modified for lower computational complexity than if it were employed as an algorithm for a full experiment.
    public boolean BFS(){
        //BFS ignores reward; it just searches for a goal condition satisfying state
        DeterministicPlanner planner = new BFS(domain, goalCondition, hashingFactory);
        try{planner.planFromState(initialState);}
        catch(PlanningFailedException e){return false;}
        return true;
    }

    /*
    * Given the current task, rf, tf and etc. return a pair with the QLearning agent and EpisodeAnalysis that contains all the 
    * states, actions and reward tuples.
    *
    */
    public LearningPair QPolicyEval(String outputPath, int numSteps, double gamma){
        int numTrials = 1;
        if(!outputPath.endsWith("/")){
            outputPath = outputPath + "/";
        }
        //domain, reward function, terminal function, gamma, hashingfactory, qinit, learning rate, learning policy, maxSteps
        //OOMDPPlanner planner = new QLearning(domain, rf, tf, 0.99, hashingFactory, 0.0, 0.1, numSteps);
        //planner.planFromState(initialState);


        LearningAgent agent = new QLearning(domain, rf, tf, gamma, hashingFactory, 0, 0.1);
        //ExponentialDecayLR lr = new ExponentialDecayLR(0.9, .5, .1);
        //((QLearning) agent).setLearningRateFunction(lr);
     /*
        if(rf instanceof RSGridWorldPotentialFunction){
            if(((RSGridWorldPotentialFunction) rf).obstacleName.equals(RSGridWorld.OBSTACLEGOAL)){
                RSGridWorldLearningRate lr = new RSGridWorldLearningRate(((RSGridWorldPotentialFunction)rf).av);
                lr.setAgent((QLearning)agent);
                ((QLearning) agent).setLearningRateFunction(lr);
            }
        }
        else if(rf instanceof RSPotentialShapedRF)
            for(int j = 0; j < ((RSPotentialShapedRF)this.rf).potentialFunction.size(); j++){
                RSGridWorldPotentialFunction rspf = ((RSGridWorldPotentialFunction)((RSPotentialShapedRF)this.rf).potentialFunction.get(j));
                if((rspf.obstacleName.equals(RSGridWorld.OBSTACLEGOAL))){
                    RSGridWorldLearningRate lr = new RSGridWorldLearningRate(rspf.av);
                    lr.setAgent((QLearning)agent);
                    ((QLearning) agent).setLearningRateFunction(lr);
                    break;
                }
            }
       */
        ((BiasedPolicy)p).setPlanner(((OOMDPPlanner)agent));
        if(rf instanceof RSGridWorldRewardFunction )
            ((BiasedPolicy)p).setDegenerate(true);
        else{
            ((BiasedPolicy)p).setDegenerate(false);
            //ValueFunctionInitialization vfi = new RSValueFunctionInit((BiasedPolicy)p);
            //RewardFunction trf = new RSGridWorldRewardFunction(false,gwdg);
            //agent = new QLearning(domain,rf,tf,gamma,hashingFactory,vfi,0.1,p,1);
        }
        ((BiasedPolicy)p).setEpsilon(0.05);
        ((QLearning)agent).setLearningPolicy(p);

        //p = new RSGreedyQPolicy((QComputablePlanner)agent);
        //EpisodeAnalysis ea = agent.runLearningEpisodeFrom(initialState, numSteps);
        int j = 0;
        int stepsSoFar = 0;
        EpisodeAnalysis eaMerged = null;
        while(stepsSoFar < numSteps){
            if(rf instanceof RSPotentialShapedRF)
                ((RSPotentialShapedRF)rf).reset();
            EpisodeAnalysis ea = ((QLearning)agent).runLearningEpisodeFrom(this.initialState, numSteps);//p.evaluateBehavior(initialState, rf, tf,numSteps-stepsSoFar);
            stepsSoFar += ea.numTimeSteps();
            if(stepsSoFar < numSteps)
                numTrials+=1;
            if(eaMerged == null)
                eaMerged = ea;
            else
                eaMerged.appendAndMergeEpisodeAnalysis(ea);
            j++;
        }
        eaMerged.writeToFile(String.format("%se%03d", outputPath, 1), sp);
        double totalRwd =0;
        for(int i = 1; i < eaMerged.numTimeSteps(); i++)
            totalRwd += eaMerged.getReward(i);
        System.out.println("Trained for " + eaMerged.numTimeSteps() + " with a total of " + totalRwd + " reward");
        ((BiasedPolicy)p).setEpsilon(0.1);

        ((QLearning)agent).setLearningPolicy(p);
        return new LearningPair(eaMerged, agent);
    }

    /*
     * A learning strategy in which the agent trains in a task until all (s,a) pairs
     * have converged within delta of their value.
     */
    public LearningPair QPolicyEval(String outputPath, double delta, double gamma){
        int numTrials = 100;
        if(!outputPath.endsWith("/")){
            outputPath = outputPath + "/";
        }
        HashMap<Coordinates, Double> QvalsLearned = new HashMap<Coordinates,Double>();
        //domain, reward function, terminal function, gamma, hashingfactory, qinit, learning rate, learning policy, maxSteps
        //OOMDPPlanner planner = new QLearning(domain, rf, tf, 0.99, hashingFactory, 0.0, 0.1, numSteps);
        //planner.planFromState(initialState);
        double averageSteps = 0;
        LearningAgent agent = new QLearning(domain, rf, tf, gamma, hashingFactory, 0., 0.1);
        EpisodeAnalysis eaMerged = null;

        for(int k = 0; k < numTrials; k++){
            agent = new QLearning(domain, rf, tf, gamma, hashingFactory, 0., 0.1);
            ExponentialDecayLR lr = new ExponentialDecayLR(0.9, .99, .1);
            ((QLearning) agent).setLearningRateFunction(lr);

            ((BiasedPolicy)p).setPlanner(((OOMDPPlanner)agent));
            ((BiasedPolicy)p).setDegenerate(true);
            ((QLearning)agent).setLearningPolicy(p);
            ((BiasedPolicy)p).setEpsilon(0.1);


            int j = 0;
            int stepsSoFar = 0;
            eaMerged = null;
            int numSteps = 20000;
            QvalsLearned.clear();

            while(true){
                boolean converged = true;
                EpisodeAnalysis ea = ((QLearning)agent).runLearningEpisodeFrom(this.initialState, numSteps);//p.evaluateBehavior(initialState, rf, tf,numSteps-stepsSoFar);
                stepsSoFar += ea.numTimeSteps();
                double change = 0.0;
                if(eaMerged == null)
                    eaMerged = ea;
                else
                    eaMerged.appendAndMergeEpisodeAnalysis(ea);
                for(int i = 1; i < eaMerged.numTimeSteps() -1; i++){
                    State curState = eaMerged.getState(i);
                    ObjectInstance agentPrev = curState.getObjectsOfClass(RSGridWorld.CLASSAGENT).get(0);
                    int prevax = agentPrev.getIntValForAttribute(RSGridWorld.ATTX);
                    int prevay = agentPrev.getIntValForAttribute(RSGridWorld.ATTY);
                    Coordinates temp = new Coordinates(prevax, prevay,eaMerged.getAction(i).actionName());
                    double qval = ((QLearning)agent).getQ(eaMerged.getState(i), eaMerged.getAction(i)).q;
                    if(!QvalsLearned.containsKey(temp)){
                        QvalsLearned.put(temp, qval);
                        converged = false;
                    }
                    else{
                        change += (Math.abs(QvalsLearned.get(temp) - qval));
                        QvalsLearned.put(temp, qval);
                        if(change <= delta)
                            converged &= true;
                        else
                            converged &= false;
                        change = 0.;
                    }
                }
                if(converged)
                    break;
                //if(QvalsLearned.size() < ((gwdg.getWidth() * gwdg.getHeight() - 1) * 4) && !(change <= delta))
                //    j++;
                //else
                //    break;
            }
            eaMerged.writeToFile(String.format("%se%03d", outputPath, 1), sp);
            double totalRwd =0;
            int numDef = 0;
            for(int i = 1; i < eaMerged.numTimeSteps(); i++){
                totalRwd += eaMerged.getReward(i);
            }
            averageSteps += eaMerged.numTimeSteps();
            System.out.println("Trained for " + eaMerged.numTimeSteps() + " steps, " + j + "episodes with a total of " + totalRwd + " reward");
            ((BiasedPolicy)p).setEpsilon(0.1);
        }
        System.out.println("Average number of steps to convergence for 10 epochs: " + averageSteps/numTrials);
        ((QLearning)agent).setLearningPolicy(p);
        return new LearningPair(eaMerged, agent);
    }
    /*
    * yet another QPolicyEval method. This is specifically used for incremental training,  
    * used for determining the optimal number of training steps 
    */
    public LearningPair QPolicyEval(LearningPair lp, int numSteps, double epsilon){
        int numTrials = 1;
        //LearningAgent agent = new QLearning(domain, rf, tf, gamma, hashingFactory, 0, 0.1);
        LearningAgent agent = (LearningAgent)lp.la;
        if(rf instanceof RSGridWorldPotentialFunction){
            if(((RSGridWorldPotentialFunction) rf).obstacleName.equals(RSGridWorld.OBSTACLEGOAL)){
                RSGridWorldLearningRate lr = new RSGridWorldLearningRate(((RSGridWorldPotentialFunction)rf).av);
                lr.setAgent((QLearning)agent);
                ((QLearning) agent).setLearningRateFunction(lr);
            }
        }
        else if(rf instanceof RSPotentialShapedRF)
            for(int j = 0; j < ((RSPotentialShapedRF)this.rf).potentialFunction.size(); j++){
                RSGridWorldPotentialFunction rspf = ((RSGridWorldPotentialFunction)((RSPotentialShapedRF)this.rf).potentialFunction.get(j));
                if((rspf.obstacleName.equals(RSGridWorld.OBSTACLEGOAL))){
                    RSGridWorldLearningRate lr = new RSGridWorldLearningRate(rspf.av);
                    lr.setAgent((QLearning)agent);
                    ((QLearning) agent).setLearningRateFunction(lr);
                    break;
                }
            }
        
        ((BiasedPolicy)p).setPlanner(((OOMDPPlanner)agent));
        if(rf instanceof RSGridWorldRewardFunction)
            ((BiasedPolicy)p).setDegenerate(true);
        else{
            ((BiasedPolicy)p).setDegenerate(false);
        }
        ((QLearning)agent).setLearningPolicy(p);
        ((BiasedPolicy)p).setEpsilon(epsilon);

        int j = 0;
        int stepsSoFar = 0;
        EpisodeAnalysis eaMerged = (EpisodeAnalysis)lp.ep;
        while(stepsSoFar < numSteps){
            if(rf instanceof RSPotentialShapedRF)
                ((RSPotentialShapedRF)rf).reset();
            EpisodeAnalysis ea = ((QLearning)agent).runLearningEpisodeFrom(this.lastState, numSteps);
            stepsSoFar += ea.numTimeSteps();
            if(stepsSoFar < numSteps)
                numTrials+=1;
            lastState = ea.getState(ea.numTimeSteps()-1);
            if(tf.isTerminal(lastState))
                lastState = initialState;
            if(eaMerged == null)
                eaMerged = ea;
            else
                eaMerged.appendAndMergeEpisodeAnalysis(ea);
            j++;
        }
        double totalRwd =0;
        for(int i = 1; i < eaMerged.numTimeSteps(); i++)
            totalRwd += eaMerged.getReward(i);
        ((BiasedPolicy)p).setEpsilon(0.1);

        ((QLearning)agent).setLearningPolicy(p);
        return new LearningPair(eaMerged, agent);
    }

    public void ValueIterationExample(String outputPath){

        if(!outputPath.endsWith("/")){
            outputPath = outputPath + "/";
        }


        OOMDPPlanner planner = new ValueIteration(domain, rf, tf, 0.99, hashingFactory,0.001, 100);
        planner.planFromState(initialState);

        //create a Q-greedy policy from the planner
        Policy p = new GreedyQPolicy((QComputablePlanner)planner);

        //record the plan results to a file
        p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "planResult", sp);

        //visualize the value function and policy
        this.valueFunctionVisualize((QComputablePlanner)planner, p);
    }

    public void visualize(String outputPath){
        Visualizer v = RSGridWorldVisualizer.getVisualizer(gwdg.getMap());
        EpisodeSequenceVisualizer evis = new EpisodeSequenceVisualizer(v, domain, sp, outputPath);
    }

    public void valueFunctionVisualize(QComputablePlanner planner, Policy p){
        List <State> allStates = StateReachability.getReachableStates(initialState,
            (SADomain)domain, hashingFactory);
        LandmarkColorBlendInterpolation rb = new LandmarkColorBlendInterpolation();
        rb.addNextLandMark(0., Color.RED);
        rb.addNextLandMark(1., Color.BLUE);

        StateValuePainter2D svp = new StateValuePainter2D(rb);
        svp.setXYAttByObjectClass(RSGridWorld.CLASSAGENT, RSGridWorld.ATTX,
        RSGridWorld.CLASSAGENT, RSGridWorld.ATTY);

        PolicyGlyphPainter2D spp = new PolicyGlyphPainter2D();
        spp.setXYAttByObjectClass(RSGridWorld.CLASSAGENT, RSGridWorld.ATTX,
            RSGridWorld.CLASSAGENT, RSGridWorld.ATTY);
        spp.setActionNameGlyphPainter(RSGridWorld.ACTIONNORTH, new ArrowActionGlyph(0));
        spp.setActionNameGlyphPainter(RSGridWorld.ACTIONSOUTH, new ArrowActionGlyph(1));
        spp.setActionNameGlyphPainter(RSGridWorld.ACTIONEAST, new ArrowActionGlyph(2));
        spp.setActionNameGlyphPainter(RSGridWorld.ACTIONWEST, new ArrowActionGlyph(3));
        spp.setRenderStyle(PolicyGlyphRenderStyle.DISTSCALED);

        ValueFunctionVisualizerGUI gui = new ValueFunctionVisualizerGUI(allStates, svp, planner);
        gui.setSpp(spp);
        gui.setPolicy(p);
        gui.setBgColor(Color.GRAY);
        gui.initGUI();
    }

    public List<State> getStates(){
        return StateReachability.getReachableStates(initialState, (SADomain)domain, hashingFactory);
    }
    public OOMDPPlanner getPlanner(){
        OOMDPPlanner omp = new ValueIteration(domain, rf, tf, 0.99, hashingFactory,0.001, 100);
        return omp;
    }
    public Policy getPolicy(){
        return p;
    }
    public OOMDPPlanner getSarsaPlanner(int stepsTraining){
        //discount= 0.99; initialQ=0.0; learning rate=0.5; maxEpisodeSize = x steps; lambda=1.0
        OOMDPPlanner planner = new SarsaLam(domain, rf, tf, 0.99, hashingFactory, 0., 0.5, stepsTraining, 1.0);
        planner.planFromState(initialState);
        return planner;
    }
    //returns a PotentialShaped reward function based on a state vector of states in a previous episode
    protected RSPotentialShapedRF getPotentialRF(double discount, List<State> allStates, OOMDPPlanner planner, String obname){
        RSGridWorldAgentSpaceBuilder asb = null; //new RSGridWorldAgentSpaceBuilder((QComputablePlanner) planner, p);
        RSGridWorldPotentialFunction rsgwpf = null; // new RSGridWorldPotentialFunction(asb.getASTuple(allStates), obname, p);
        return new RSPotentialShapedRF(new RSGridWorldRewardFunction(gwdg), new RSGridWorldPotentialFunction[] {rsgwpf}, discount);
    }
    //returns a PotentialShaped reward function based on a state vector of states in a previous episode
    protected RSPotentialShapedRF getPotentialRF(double discount, List<List<State>> allStates, OOMDPPlanner[] planner, String[] obname, Policy[] par){
        RSGridWorldPotentialFunction[] pf = new RSGridWorldPotentialFunction[obname.length];
        for(int i = 0; i < obname.length; i++){
            RSGridWorldAgentSpaceBuilder asb = null ; //new RSGridWorldAgentSpaceBuilder((QComputablePlanner) planner[i], par[i]);
            RSGridWorldPotentialFunction rsgwpf = null; //new RSGridWorldPotentialFunction(asb.getASTuple(allStates.get(i)), obname[i], par[i]);
            pf[i] = rsgwpf;
        }
        return new RSPotentialShapedRF(new RSGridWorldRewardFunction(gwdg), pf, discount);
    }
    protected RSPotentialShapedRF getPotentialRF(double discount, List<List<State>> allStates, ArrayList<Map<String, HashMap<Coordinates, Double>>> tuple, String[] obname, Policy[] par){
        RSGridWorldPotentialFunction[] pf = new RSGridWorldPotentialFunction[obname.length];
        for(int i = 0; i < obname.length; i++){
            RSGridWorldPotentialFunction rsgwpf = null;// new RSGridWorldPotentialFunction(tuple.get(i), obname[i], par[i]);
            pf[i] = rsgwpf;
        }
        return new RSPotentialShapedRF(new RSGridWorldRewardFunction(gwdg), pf, discount);
    }

    /* This method creates a graph, or several, based on some class defined metrics. This compares Qlearning and sarsaLamda on
     * the same domain, reward function, and terminal function
     */
    public void experimenterAndPlotter(String fileName, boolean fwd_advice, int trialNum){

        /**
         * Create factories for Q-learning agent
         */
        final boolean factory_fwd_advice = fwd_advice;
        LearningAgentFactory qLearningFactory = new LearningAgentFactory() {

            @Override
            public String getAgentName() {
                return "Q-learning";
            }

            @Override
            public LearningAgent generateAgent() {
                //TerminalFunction temptf = new RSGridWorldTerminalFunction(gwdg);
                OOMDPPlanner temp = new QLearning(domain, rf, tf, 0.99, hashingFactory, 0., 0.1);
                ((BiasedPolicy)p).setPlanner(temp);
                System.out.println("set degenerate policy status: " + factory_fwd_advice);
                //((BiasedPolicy)p).setDegenerate(!factory_fwd_advice); //if no transfer, biasedPolicy degenerates to ep greedy
                ((QLearning)temp).setLearningPolicy(p);
                if(rf instanceof RSPotentialShapedRF)
                    ((RSPotentialShapedRF)rf).reset();
                return (LearningAgent)temp;
            }
        };

        PrintWriter writer = null;
        try{
            if(trialNum == 0)
                writer = new PrintWriter(fileName+".csv", "UTF-8");
            else
                writer = new PrintWriter(new FileOutputStream(new File(fileName+".csv"), true)); //append 
        }
        catch(Exception e){
            e.printStackTrace();
        } finally {
            //StateGenerator sg = new ConstantStateGenerator(this.initialState);
            int numTrials = 10;
            int numSteps = 15000;
            int expRwdInt = 10; //number of steps inbetween expected reward evaluations


            //numSteps = 300000;
            //expRwdInt = 400;
            if(trialNum == 0)
                writer.println("agent,trial,step,expected_rwd,");


            //Policy pi = new BiasedPolicy();
            //((BiasedPolicy)p).setPlanner(temp);
            ((BiasedPolicy)p).setDegenerate(!fwd_advice); //if no transfer, biasedPolicy degenerates to ep greedy
            
            /*
             * The following code evaluates the optimal policy after x steps of learning
            */
            RSGridWorldRewardFunction baserf = new RSGridWorldRewardFunction(gwdg);


            for(int i = 0; i < numTrials; i++){
                if(rf instanceof RSPotentialShapedRF)
                    ((RSPotentialShapedRF)rf).reset();
                int stepsRemaining = numSteps;
                State firstState = this.initialState;//sg.generateState();
                State lastState = firstState;
                LearningAgent agent = qLearningFactory.generateAgent();
                //ExponentialDecayLR lr = new ExponentialDecayLR(0.9, .5, .1);
                //((QLearning) agent).setLearningRateFunction(lr);
                ((QLearning)agent).setLearningPolicy(p);

        /*if(rf instanceof RSGridWorldPotentialFunction){
            if(((RSGridWorldPotentialFunction) rf).obstacleName.equals(RSGridWorld.OBSTACLEGOAL)){
                RSGridWorldLearningRate lr = new RSGridWorldLearningRate(((RSGridWorldPotentialFunction)rf).av);
                lr.setAgent((QLearning)agent);
                ((QLearning) agent).setLearningRateFunction(lr);
            }
        }
        else if(rf instanceof RSPotentialShapedRF){
           for(int j = 0; j < ((RSPotentialShapedRF)this.rf).potentialFunction.size(); j++){
                RSGridWorldPotentialFunction rspf = ((RSGridWorldPotentialFunction)((RSPotentialShapedRF)this.rf).potentialFunction.get(j));
                //if((rspf.obstacleName.equals(RSGridWorld.OBSTACLEGOAL))){
                    RSGridWorldLearningRate lr = new RSGridWorldLearningRate(rspf.av);
                    lr.setAgent((QLearning)agent);
                    ((QLearning) agent).setLearningRateFunction(lr);
                    break;
                }
                //}
            }
*/
                ((BiasedPolicy)p).setPlanner((OOMDPPlanner)agent);
                EpisodeAnalysis ea = new EpisodeAnalysis();
                int numEvalSteps = 200;
                //int numEvalSteps = 1000;
                int trials = 1;
				expRwdInt = 10;
                System.out.println("On trial " + i + " of " + numTrials);
                System.out.println("On global trial: " + trialNum);
                double lastRwd = 0.;
                while(stepsRemaining > 0){
                    //System.out.println("Steps remaining: " + stepsRemaining);
                    int evalNum = 0;
                   // ((BiasedPolicy)p).setPlanner(((OOMDPPlanner)agent));

                    if(tf.isTerminal(lastState)){
                        if(rf instanceof RSPotentialShapedRF)
                            ((RSPotentialShapedRF)rf).reset();
                        lastState = this.initialState;
                    }
                     ea = ((QLearning)agent).runLearningEpisodeFrom(lastState,10);

                     //evaluate policy here
                    int j = 0;
                    EpisodeAnalysis eaMerged = null;
                    int trialsSoFar = 0; //holds the number of steps executed in policy evaluation
                    
                    ((BiasedPolicy)p).setEpsilon(0.0);
                    //((BiasedPolicy)p).setDegenerate(true);
                    //double oldPot = ((RSPotentialShapedRF)rf).setLast(0.0);
                    while(evalNum < trials){
                        RewardFunction temprf = rf;
                        if(rf instanceof RSPotentialShapedRF){
                            temprf = new RSPotentialShapedRF((RSPotentialShapedRF)rf);
                            ((RSPotentialShapedRF)temprf).lastpot = 0.0;
                            ((RSPotentialShapedRF)temprf).salastpot = 0.0;
                        }
                        EpisodeAnalysis eapi = p.evaluateBehavior(initialState, temprf, tf,numEvalSteps);
                        //stepsSoFar += eapi.numTimeSteps();
                        if(eaMerged == null)
                            eaMerged = eapi;
                        else
                            eaMerged.appendAndMergeEpisodeAnalysis(eapi);
                        evalNum++;
                    }
                    //((RSPotentialShapedRF)rf).setLast(oldPot);
                    ((BiasedPolicy)p).setEpsilon(.1);
                    //((BiasedPolicy)p).setDegenerate(!fwd_advice);
                    double totalRwd = 0;
                    for(int d = 0; d < eaMerged.numTimeSteps()-1; d++){
                        //totalRwd += eaMerged.getReward(d); //remove the -1 above if using this?
                        totalRwd += baserf.reward(eaMerged.getState(d),eaMerged.getAction(d),eaMerged.getState(d + 1));
                    }
                    totalRwd/=trials;
                    //System.out.println("On evaluation: " + evalNum + " the policy produced " + totalRwd + " total reward.");

                     //write to file here based on timestep and trial

                    lastState = ea.getState(ea.maxTimeStep());
                    //System.out.println("Steps remaining: " + stepsRemaining);
                    stepsRemaining -= (expRwdInt);
                    writer.println("Q-Learning,"+i+","+(numSteps-stepsRemaining)+","+totalRwd+",");
                    if(evalNum % 1 == 0){
                        //System.out.println("Eval: " + evalNum);
                        //if(totalRwd < lastRwd)
                            //printTargetHeatMap(eaMerged);
                       //lastRwd = totalRwd;
                    }
                }
                writer.flush();
            }


            /*
             * Eval learning: record reward accumulated in every learning episode
             */
/*
            for(int i = 0; i < numTrials; i++){
                State firstState = this.initialState;
                State lastState = firstState;
                LearningAgent agent = qLearningFactory.generateAgent();
                ((QLearning)agent).setLearningPolicy(p);
                ((BiasedPolicy)p).setPlanner((OOMDPPlanner)agent);
                EpisodeAnalysis ea = new EpisodeAnalysis();
                int episodeNum = 0;
                int totalEpisodes = 500;
                System.out.println("On trial " + i + " of " + numTrials);
                while(totalEpisodes > episodeNum){
                    //if(rf instanceof RSPotentialShapedRF )
                    //    ((RSPotentialShapedRF)rf).clearBaseOccGrid();
                    //else if (rf instanceof RSGridWorldRewardFunction)
                    //    ((RSGridWorldRewardFunction)rf).clearMusicList();
					
                     ea = ((QLearning)agent).runLearningEpisodeFrom(this.initialState, expRwdInt);
                     ((RSPotentialShapedRF)rf).reset();

                    ((BiasedPolicy)p).setPlanner(((OOMDPPlanner)agent));

                    double totalRwd = 0;
                    for(int d = 0; d < ea.numTimeSteps()-1; d++){ //eamerged for policy evaluation
                        //totalRwd += ea.getReward(d);
                        totalRwd += baserf.reward(ea.getState(d),ea.getAction(d),ea.getState(d + 1));
                    }
                     //write to file here based on timestep and trial
                    writer.println("Q-Learning,"+i+","+(episodeNum)+","+totalRwd+",");
                    episodeNum++;
                }
            }*/
            writer.close();
        }  
    }

    public void runThreadedExperimenter(String baseFileName, String outputFile, String sourcePath, String sourceFile, String targetPath, String targetFile){
        double gamma_train = 0.7;
        double gamma_eval = 0.9;
        double epsilon = 0.1;
        int max_training = 1500;
        int step = 10;
        int targetTrials = 10;
        RewardShape example;
        int totTrials = 0;
        List<ThreadedExperiment> tlist = new ArrayList<ThreadedExperiment>();
        while( totTrials < targetTrials){
            ThreadedExperiment te = new ThreadedExperiment("Thread", domain, tf, rf, hashingFactory,
                                                           (burlap.oomdp.core.State) initialState, (baseFileName + totTrials), totTrials, p, gwdg);
            
            te.setparams(baseFileName, outputFile, sourcePath, sourceFile, targetPath, targetFile, step, gamma_train, gamma_eval, epsilon, max_training);
            te.start();
            
            tlist.add(te);
            totTrials++;
            if(totTrials == targetTrials){
                try{
                    for(ThreadedExperiment temp : tlist)
                        te.t.join();
                        Thread.sleep(1000);
                }
                catch(InterruptedException e){
                    System.out.println("InterruptedException prior to merge");
                }
                if(targetTrials > 1){
                    //MergeAndParseCSV csv = new MergeAndParseCSV(baseFileName, targetTrials);
                    //csv.run();
                }
            }
        }

    }
    //quick function to generate random domains to file for baselines.
    //not automated, should be manually filled
    public static void generateDomainsToFile(String[] obstacles){
        Random rand = new Random();
        for(int i = 1; i < 2; i++){
            int randSize = rand.nextInt((15 - 2) + 1) + 2;
            int randomNum = rand.nextInt((int)Math.round((randSize*randSize*.25)) + 1);
            String out = "domains/target/";
            RewardShape example = new RewardShape(obstacles, 50, 30, 30, out + i + ".dat");
        }
    }
    public void QLearningExample(String outputPath){

        if(!outputPath.endsWith("/")){
            outputPath = outputPath + "/";
        }

        //creating the learning algorithm object; discount= 0.99; initialQ=0.0; learning rate=0.9
        LearningAgent agent = new QLearning(domain, rf, tf, 0.99, hashingFactory, 0., 0.9);

        //run learning for 100 episodes
        for(int i = 0; i < 100; i++){
            EpisodeAnalysis ea = agent.runLearningEpisodeFrom(initialState);
            ea.writeToFile(String.format("%se%03d", outputPath, i), sp);
            System.out.println(i + ": " + ea.numTimeSteps());
        }

    }
}
