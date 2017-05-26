# -*- coding: utf-8 -*-
"""
Created on May, 25th, 08:29 2017.

@author: Felipe Leno
Source for running the experiments. All the parameters will be here specified and 
all the relevant sources will be called
"""

import argparse
import sys

import csv
import random
from domain.environment import GridWorld
from domain.task import Task
import os
#from cmac import CMAC


#from agents.agent import Agent

  


def get_args():
    """Arguments for the experiment
            --task_path: Path for the file defining the final target task
            --algorithm: Learning algorithm (subclass of Agent)
            --learning_episodes: Maximum number of learning episodes to be executed
            --type_evaluation: Type of evaluation (per episodes or per steps)
            --evaluation_interval: interval of episodes for evaluation (episodes or steps)
            --evaluation_duration: Number of evaluation episodes
            --seed: Seed for random procedures
            --log_folder: output folder
            --temp_folder: folder to be possibly used by the algorithm
            --curriculum_alg: Algorithm for Curriculum generation
            --graphics: Shows domain illustration.
            
    
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-t','--task_path', default='./tasks/target.task')
    parser.add_argument('-a','--algorithm',  default='Dummy') 
    parser.add_argument('-e','--learning_time',type=int, default=10000)
    parser.add_argument('-te','--type_evaluation',choices=['episode','steps'], default='steps')
    parser.add_argument('-i','--evaluation_interval',type=int, default=100)
    parser.add_argument('-d','--evaluation_duration',type=int, default=2)
    parser.add_argument('-s','--seed',type=int, default=12345)
    parser.add_argument('-l','--log_folder',default='./log/')
    parser.add_argument('-tf','--temp_folder',default='./temp/')
    parser.add_argument('-et','--end_trials',type=int, default=50)
    parser.add_argument('-ca','--curriculum_alg',default='NoneCurriculum')
    parser.add_argument('-g','--graphics',type=bool,default=False)
     

    return parser.parse_args()

def keep_training(curriculum,episodes,totalSteps,parameter):
    """
        Given the Curriculum, number of episodes, total number of steps, and parameters,
        defines if the agent still needs to train
    """
    current = episodes if parameter.type_evaluation == 'episode' else totalSteps
    #So far, only compares total training time.
    return current <= parameter.learning_time
        
def evaluate_now(episodes,totalSteps,parameter): 
    """
        Defines if the evaluation should be carried out now
    """
    current = episodes if parameter.type_evaluation == 'episode' else totalSteps
    #If the number is divisible by 2
    return current % parameter.evaluation_interval == 0
    
    
def build_objects():
    """Builds the objects specified in the argument and returns them in the following order:
        Agent, CurriculumAlg    
            
    """
     
    parameter = get_args()
    
    agentName = getattr(parameter,"algorithm")
    print "Algorithm: "+agentName
    try:
            AgentClass = getattr(
               __import__('agents.' + (agentName).lower(),
                          fromlist=[agentName]),
                          agentName)
    except ImportError as error:
            print error
            sys.stderr.write("ERROR: missing python module: " +agentName + "\n")
            sys.exit(1)
        
    AGENT = AgentClass(seed=parameter.seed)
 
    #ok AGENT
        
    

    curriculumName = getattr(parameter,"curriculum_alg")
    print "Curriculum: "+curriculumName
    try:
            CurriculumClass = getattr(
               __import__('curriculum.' + (curriculumName).lower(),
                          fromlist=[curriculumName]),
                          curriculumName)
    except ImportError as error:
            print error
            sys.stderr.write("ERROR: missing python module: " +curriculumName + "\n")
            sys.exit(1)
        
    CURRICULUM = CurriculumClass(seed=parameter.seed)
    
    return AGENT,CURRICULUM
    

def main():
    parameter = get_args()
    print parameter
   
    #Folder for temp files
    workFolder = parameter.temp_folder + parameter.algorithm + '/'
    

    
    for trial in range(1,parameter.end_trials+1):
        #Folder for results
        logFolder = parameter.log_folder + parameter.algorithm+"-"+parameter.curriculum_alg
        if not os.path.exists(logFolder):
                os.makedirs(logFolder)
        logFolder = logFolder + "/_0_"+str(trial)+"_AGENT_1_RESULTS"
        
        #Output Files
        eval_csv_file = open(logFolder + "_eval", "wb")
        eval_csv_writer = csv.writer(eval_csv_file)
        eval_csv_writer.writerow((parameter.type_evaluation,"steps_completed","reward"))
        eval_csv_file.flush()
        
        print('***** %s: Start Trial' % str(trial))            
        random.seed(parameter.seed+trial)
        agent,curriculum = build_objects()
        
        #links the curriculum algorithm with the learning agent
        curriculum.set_agent(agent)
        
        
        #Load target Task
        target_task = Task(filePath=parameter.task_path)
        environment_target = GridWorld(treasures=1,pits = target_task.num_pits(),fires = target_task.num_fires(),
                                    sizeX = target_task.get_sizeX(),sizeY = target_task.get_sizeY(),
                                    taskState = target_task.init_state(), limitSteps = 200)
        
        #Generate Curriculum for target task
        curriculum.generate_curriculum(target_task, workFolder)
  
        #While there is still tasks to be learned
        while not curriculum.empty_curriculum():
            task = curriculum.draw_task()

            #Initiate task
            environment = GridWorld(treasures=1,pits = task.num_pits(),fires = task.num_fires(),
                                    sizeX = task.get_sizeX(),sizeY = task.get_sizeY(),taskState = task.init_state(), limitSteps = 200)
            environment.start_episode()
            
            
            
            episodes = 0 
            totalSteps = 0
            terminal = False
            #Verifies termination condition
            while keep_training(curriculum,episodes,totalSteps,parameter):
                #Check if it is time to policy evaluation
                if evaluate_now(episodes,totalSteps,parameter):
#--------------------------------------- Policy Evaluation---------------------------------------------
                    agent.set_exploring(False)
                    agent.connect_env(environment_target)
                    stepsToFinish = 0
                    #Executes the number of testing episodes specified in the parameter
                    for eval_episode in range(1,parameter.evaluation_duration+1):
                        curGamma = 1.0
                        sumR = 0
                        eval_step = 0
                        environment_target.start_episode()

                        terminal_target= False
                                                
                        while not terminal_target:
                            eval_step += 1
                            state = environment_target.get_state()
                            environment_target.act(agent.select_action(state))
                            
                            #Process state transition
                            statePrime,action,reward = environment_target.step()        
                            sumR += reward * curGamma
                            curGamma = curGamma * agent.gamma      
                            
                            terminal_target = environment_target.is_terminal_state()
                        stepsToFinish += eval_step
                        
                    stepsToFinish = float(stepsToFinish) / parameter.evaluation_duration
                                         
                    time = episodes if parameter.type_evaluation == 'episode' else totalSteps
                    eval_csv_writer.writerow((time,"{:.2f}".format(stepsToFinish),"{:.15f}".format(sumR)))
                    eval_csv_file.flush()
                    agent.set_exploring(True)  
                    #print("*******Eval OK: EP:"+str(episodes)+" Steps:"+str(totalSteps)+" - Duration: "+str(stepsToFinish))
#-----------------------------------End Policy Evaluation---------------------------------------------
                #One larning step is performed
                totalSteps += 1
                agent.connect_env(environment)
                state = environment.get_state()
                environment.act(agent.select_action(state))
                #Process state transition
                statePrime,action,reward = environment.step()   
                agent.observe_reward(state,action,statePrime,reward)
                
                terminal = environment.is_terminal_state()
                
                #If the agent reached a terminal state, initiates the new episode
                if terminal:
                    episodes += 1
                    environment.start_episode()
                    agent.finish_episode()
            agent.finish_learning()
                
                
        #Close result files
        eval_csv_file.close()
                    
    
    
    
    

if __name__ == '__main__':
    main()
