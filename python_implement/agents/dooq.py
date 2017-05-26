# -*- coding: utf-8 -*-
"""
Created on Mon Nov 21 13:47:01 2016
DOO-Q implementation
@author: Felipe Leno
"""

import random
import math

from .agent import Agent
from common_features import Agent_Utilities

import copy
import actions

class DOOQ(Agent):
    
    gamma = None
    T = None
    qTable = None
    functions = None
    policy = None
    friends = None
    

    def __init__(self, seed=12345,numAg = 3,gamma=0.9,T=0.4):
        self.sortFriends = None
        self.gamma = gamma
        self.T = T
        self.functions = Agent_Utilities()
        self.qTable = {}
        self.policy = {}
        self.friends = None
        super(DOOQ, self).__init__(seed=seed,numAg = numAg)
        
        
    def initiate_agent_refs(self,numAg,seed):
        """ Create the references to be executed by experiment.py """
        agents = []
        for i in range(numAg):        
            agents.append(copy.deepcopy(self))
            agents[i].sortFriends = True
      
        
        return agents

        
        
    def get_proc_state(self,agentIndex):
        """ Returns a processed version of the current state """
        return self.environment.get_state(agentIndex,self.sortFriends)
            
        

            
    
    def select_action(self, state,agentIndex):
        """ When this method is called, the agent executes an action. """
        
        if self.blind_state(state):        
            return random.choice(self.getPossibleActions())
        #Computes the best action for each agent        
        if self.exploring:
               action =  self.exp_strategy(state)
           #Else the best action is picked
        else:
            action = self.policy_check(state)
        
        return action
    def blind_state(self,state):
        """Returns if the agent can see anything"""
        for i in range(len(state)):
            if state[i] != float('inf'):
                return False
        return True
        
        
    def policy_check(self,state):
        """In case a fixed action is included in the policy cache, that action is returned
        else, the maxQ action is returned"""
        if state in self.policy:
           return self.policy[state]
        return self.max_Q_action(state)
        
        
    def max_Q_action(self,state):
        """Returns the action that corresponds to the highest Q-value"""
        actions = self.getPossibleActions()
        v,a =  self.functions.get_max_Q_value_action(self.qTable,state,actions,self.exploring)
        return a
    def get_max_Q_value(self,state):
        """Returns the maximum Q value for a state"""
        actions = self.getPossibleActions()
        v,a =  self.functions.get_max_Q_value_action(self.qTable,state,actions,self.exploring)
        return v
        
        
        
    def exp_strategy(self,state):
        """Returns the result of the exploration strategy"""
        useBoltz = False        
        allActions = self.getPossibleActions()
        if useBoltz:
            #Boltzmann exploration strategy
            valueActions = []
            sumActions = 0
            
            for action in allActions:
                qValue = self.qTable.get((state,action),0.0)
                vBoltz = math.pow(math.e,qValue/self.T)
                valueActions.append(vBoltz)
                sumActions += vBoltz
            
            probAct = []
            for index in range(len(allActions)):
                probAct.append(valueActions[index] / sumActions)
            
            rndVal = random.random()
            
            sumProbs = 0
            i=-1
            
            while sumProbs <= rndVal:
                i = i+1
                sumProbs += probAct[i]
            
            return allActions[i]
        else:
            prob = random.random()
            if prob <= 0.1:
                return random.choice(allActions)
            return self.max_Q_action(state)
           

    
    def get_Q_size(self,agentIndex):
        """Returns the size of the QTable"""
        return len(self.qTable)
        
    
    def observe_reward(self,state,action,statePrime,reward,agentIndex):
        """Performs the standard Q-Learning Update"""
        if self.exploring:
            qValue= self.qTable.get((state,action),None)
            V = self.get_max_Q_value(statePrime)        
            if qValue is None:
                newQ = reward
            else:
                newQ = max(qValue, reward + self.gamma * V )
                if newQ > qValue:
                    #Check if the policy needs to be updated
                    if self.functions.check_various_max_Q(self.qTable,state,self.getPossibleActions()):
                        self.policy[state] = action
                    else:
                        if state in self.policy:
                            del self.policy[state]    
            self.qTable[(state,action)] = newQ
        
        
    
    def getPossibleActions(self):
        """Returns the possible actions"""
        
        return actions.all_agent_actions()

 