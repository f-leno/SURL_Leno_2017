import RSGridWorld.*;

/* Threaded class to aid experiments of larger (domain) size.
 * Create mulitple threads, all piping state information into a csv file
 * which can then be compiled into a single file, averaged, and plotted.
 * 
 * Author Maxwell J Svetlik
 */

import java.util.*;
import java.io.PrintWriter;
import java.io.File;
import java.io.FileOutputStream;

import burlap.oomdp.auxiliary.StateGenerator;
import burlap.oomdp.core.*;
import burlap.oomdp.core.State;
import burlap.behavior.singleagent.*;
import burlap.behavior.singleagent.learning.*;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.behavior.singleagent.auxiliary.performance.*;
import burlap.behavior.singleagent.learning.*;
import burlap.behavior.singleagent.learning.tdmethods.*;
import burlap.oomdp.auxiliary.common.ConstantStateGenerator;
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
import burlap.oomdp.singleagent.RewardFunction;


public class ThreadedExperiment extends Thread {
   public Thread t;
   private String threadName;
   private Domain domain;
   private String filename;
   private TerminalFunction tf;
   private RewardFunction rf;
   private DiscreteStateHashFactory hashingFactory;
   private burlap.oomdp.core.State initialState;
   private int numEpisodes;
   private Policy p;
   private RSGridWorld gwdg;

   //params
   private String baseline;
   private String outputFile;
   private String sourceFile;
   private String targetFile;
   private int step;
   private double gamma_train;
   private double gamma_eval;
   private double epsilon;
   private int max_training;
   private String targetPath;
   private String sourcePath;
   public ThreadedExperiment( String tname, Domain rsdomain, TerminalFunction rstf, RewardFunction rsrf, DiscreteStateHashFactory rshashing, 
                        burlap.oomdp.core.State rsstate, String fileOutput, int numEp, Policy p, RSGridWorld gwdg){
       domain = rsdomain;
       threadName = tname;
       filename = fileOutput;
       tf = rstf;
       rf = rsrf;
       hashingFactory = rshashing;
       initialState = rsstate;
       numEpisodes = numEp;
       this.p = p;
       this.gwdg = gwdg;
       System.out.println("Creating " +  threadName );
   }

   /*
   * baseline = filepath + name to the 'baseline' data matrix. This will be averaged
   * outputFile = filename and extension, to be used and outputted. threadnumber will be appended
   * sourceFile = filepath + name to the source task .dat file. eg domains/nstep/goal5.dat
   * targetFile = filepath + name to the target task .dat file
   */
   public void setparams(String baseline, String outputFile, String sourcePath, String sourceFile, String targetPath, String targetFile, int step,
double gamma_train, double gamma_eval, double epsilon, int max_training){
      this.baseline = baseline;
      this.outputFile = outputFile;
      this.sourceFile = sourceFile;
      this.targetFile = targetFile;
      this.step = step;
      this.gamma_train = gamma_train;
      this.gamma_eval = gamma_eval;
      this.epsilon = epsilon;
      this.max_training = max_training;
      this.targetPath = targetPath;
      this.sourcePath = sourcePath;
   }
   public void run() {
      System.out.println("Running " +  threadName );

      /*
      * grab the area under the target curve. 
      */
      DataAnalysis da_baseline = new DataAnalysis(baseline,0);
      da_baseline.parseData();
      double baseline_area = da_baseline.getArea();
      String outputPath = "data/training/";
      RewardShape example;
      LearningPair lp;
      PrintWriter writer = null;
      LearningAgent agent;
      try{
          writer = new PrintWriter(new FileOutputStream(new File(outputFile + numEpisodes + ".csv" ), false)); //append 
          writer.println("Trial,"+"Training Steps,"+"Area Gain (Source - Target),");
      }
      catch(Exception e){
          e.printStackTrace();
      } finally {
          int n = 3;
          String[] obname = new String[n];
          LearningPair[] ea_ar = new LearningPair[n];
          LearningPair ea;
          agent = new QLearning(domain, rf, tf, gamma_train, hashingFactory, 0, 0.1);
          lp = new LearningPair();
          lp.setAgent(agent);

          for(int i = 0; i < max_training; i+=step){
              example = new RewardShape(sourcePath + sourceFile, false);
              lp = example.QPolicyEval(lp, step,epsilon);

              /*
              * evaluation
              */
              int j = 0;
              example = new RewardShape("domains/nstep/fire2.dat", true);
              ea = example.QPolicyEval(outputPath,800,gamma_train);
              obname[0] = RSGridWorld.OBSTACLEFIRE;
              ea_ar[0] = ea;

              example = new RewardShape("domains/nstep/pit2.dat", true);
              ea = example.QPolicyEval(outputPath,800,gamma_train);
              obname[1] = RSGridWorld.OBSTACLEPIT;
              ea_ar[1] = ea;

              ea_ar[2] = lp;
              obname[2] = RSGridWorld.OBSTACLEGOAL;

              example = new RewardShape(targetPath + ""+ targetFile, gamma_eval, ea_ar, obname, false);
              example.experimenterAndPlotter("data/training/temp/"+targetFile + "_" + sourceFile+gamma_train+numEpisodes,true,j);
              DataAnalysis training = new DataAnalysis("data/training/temp/"+ targetFile + "_" + sourceFile+gamma_train+numEpisodes+".csv",i);
              training.parseData();
              double training_area = training.getArea();
              writer.println(numEpisodes+","+i+","+(training_area - baseline_area)+",");
              writer.flush();
          }
      
          writer.close();
      }


   }
   
   public void start(){
      System.out.println("Starting " +  threadName );
      if (t == null)
      {
         t = new Thread (this, threadName);
         t.start ();
      }
   }

    public LearningPair QPolicyEval(LearningPair lp, int numSteps, double epsilon2){
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
          EpisodeAnalysis ea = ((QLearning)agent).runLearningEpisodeFrom((burlap.oomdp.core.State)this.initialState, numSteps);
          stepsSoFar += ea.numTimeSteps();
          if(stepsSoFar < numSteps)
              numTrials+=1;
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
            int expRwdInt = 200; //number of steps inbetween expected reward evaluations


            //numSteps = 300000;
            //expRwdInt = 400;
            if(trialNum == 0)
                writer.println("agent,trial,step,expected_rwd,");

            ((BiasedPolicy)p).setDegenerate(!fwd_advice); //if no transfer, biasedPolicy degenerates to ep greedy
            
            /*
             * The following code evaluates the optimal policy after x steps of learning
            */
            RSGridWorldRewardFunction baserf = new RSGridWorldRewardFunction(gwdg);


            for(int i = 0; i < numTrials; i++){
                if(rf instanceof RSPotentialShapedRF)
                    ((RSPotentialShapedRF)rf).reset();
                int stepsRemaining = numSteps;
                burlap.oomdp.core.State firstState = this.initialState;
                burlap.oomdp.core.State lastState = firstState;
                LearningAgent agent = qLearningFactory.generateAgent();
                ((QLearning)agent).setLearningPolicy(p);

                        if(rf instanceof RSGridWorldPotentialFunction){
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

                ((BiasedPolicy)p).setPlanner((OOMDPPlanner)agent);
                EpisodeAnalysis ea = new EpisodeAnalysis();
                int numEvalSteps = 200;
                //int numEvalSteps = 1000;
                int trials = 1;
        expRwdInt = 10;
                System.out.println("On trial " + i + " of " + numTrials);
                System.out.println("On global trial: " + trialNum);
                while(stepsRemaining > 0){
                    int evalNum = 0;

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

                    while(evalNum < trials){
                        RewardFunction temprf = rf;
                        if(rf instanceof RSPotentialShapedRF)
                            temprf = new RSPotentialShapedRF((RSPotentialShapedRF)rf); 
                        EpisodeAnalysis eapi = p.evaluateBehavior(initialState, temprf, tf,numEvalSteps);
                        //stepsSoFar += eapi.numTimeSteps();
                        if(eaMerged == null)
                            eaMerged = eapi;
                        else
                            eaMerged.appendAndMergeEpisodeAnalysis(eapi);
                        evalNum++;
                    }
                    ((BiasedPolicy)p).setEpsilon(.1);
                    //((BiasedPolicy)p).setDegenerate(!fwd_advice);
                    double totalRwd = 0;
                    for(int d = 0; d < eaMerged.numTimeSteps()-1; d++){
                        //totalRwd += eaMerged.getReward(d); //remove the -1 above if using this?
                        totalRwd += baserf.reward(eaMerged.getState(d),eaMerged.getAction(d),eaMerged.getState(d + 1));
                    }
                    totalRwd/=trials;

                     //write to file here based on timestep and trial

                    lastState = ea.getState(ea.maxTimeStep());
                    //System.out.println("Steps remaining: " + stepsRemaining);
                    stepsRemaining -= (expRwdInt);
                    writer.println("Q-Learning,"+i+","+(numSteps-stepsRemaining)+","+totalRwd+",");
                }
                writer.flush();
            }
            writer.close();
        }  
    }

}
