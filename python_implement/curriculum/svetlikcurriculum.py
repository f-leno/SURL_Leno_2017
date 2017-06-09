#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon May 29 13:25:20 2017
 Implementation of the Graph-based Curriculum generation procedure proposed by Svetlik et al.
 
@author: Felipe Leno
"""

from curriculum import Curriculum
from domain import task
from domain.task import Task
from graph import Graph
import random
import os

class SvetlikCurriculum(Curriculum):
    """ This is the base class for all Curriculum Learning implementations.

    """
    thresholdTask = None #Threshold for excluding tasks from the CUrriculum
    target_task = None
    taskGraph = None
    
    taskList = None
    
    
   
    def __init__(self, seed=12345,agent=None):
        super(SvetlikCurriculum, self).__init__(seed=seed,agent=agent)
        
    
    def generate_curriculum(self,target_task, sourceFolder,workFolder,thresholdTask = 15):
        """ The Curriculum is generated by this function (stored internally) """
        self.target_task = target_task
        self.thresholdTask = thresholdTask
        
        setOfTasks = self.read_folder(sourceFolder)
        
        groups,setOfTasks = self.groupTasks(target_task,setOfTasks)     
        self.build_curriculum(target_task,groups,setOfTasks)
        
        #Defines one order for presenting tasks (no parallel task execution implemented)
        self.build_task_list(setOfTasks,target_task)
                
        print self.taskList
    
    def build_task_list(self,setOfTasks,target_task):
        """Defines one possible order for presenting the tasks"""
        state = random.getstate()
        random.seed(self.seed)
        
        self.taskList = []
        
        import copy
        tempGraph = copy.deepcopy(self.taskGraph)
        while tempGraph.has_edges():
            #Get all tasks that have solved children
            tasks = tempGraph.zero_inDegree_nodes()
            while len(tasks)>0:
                #All applicable tasks are chosen in random order;
                t = random.choice(tasks)
                #Get correct reference to task object
                origTask = next(x for x in setOfTasks if x.name==t.name) 
                self.taskList.append(origTask)
                tempGraph.remove(t)
                tasks.remove(t)
        #Includes the target task
        self.taskList.append(target_task)
        
        random.setstate(state)
            
            
    def read_folder(self,sourceFolder):
        """Reads source tasks in the given folder"""
       
        taskList = []
        for fileName in os.listdir(sourceFolder):
            if fileName.endswith(".task"):
                taskList.append(Task(sourceFolder+fileName,fileName.split('.')[0]))
        return taskList
        
        
    
    def empty_curriculum(self):
        """ Returns if the curriculum already has returned the task """
        return len(self.taskList) == 0
    
    
    def draw_task(self):
        """ Returns the previously given task"""
        task = self.taskList[0]
        self.agent.set_current_task(task)
        del self.taskList[0]
        return task
    
    def groupTasks(self,target_task,setOfTasks):
        """Groups the task according to the procedure described in their paper, the returned
        groups are sorted by transfer potential in relation to the target task."""
        groups = {} #Return, task groups
        acceptedTasks = []
        for currentTask in setOfTasks:
            transferPot = self.transfer_potential(currentTask,target_task,acceptedTasks)
            #If the task has a transfer potential greater than the threshold, it
            #is added to a group
            if transferPot > self.thresholdTask:
                taskHash = currentTask.task_features()
                #If none task with the same parameters have ben found, a new group
                # is created
                if not taskHash in groups:
                    groups[taskHash] = []
                    #Task added to its group
                groups[taskHash].append((currentTask,transferPot))
                acceptedTasks.append(currentTask)
        #Tasks inside each group are sorted by transfer potential
        for g in groups:
            #Sort by transferPot, descending order
            groups[g].sort(key=lambda x:x[1],reverse=True)  
        return groups,acceptedTasks
    def build_curriculum(self,target_task,groups,setOfTasks):
        """After the groups were defined and sorted, the curriculum is built"""
        connections = [] #list of graph connections
        graph = Graph([],directed = True)
        #Intra-group transfer
        for g in groups:
            group =  groups[g]
            #For all tasks inside each group
            for i in range(len(group)):
                currentTask = group[i][0]
                maxTransferPot = -float('inf')
                argMaxTask = None
                # get task with maximum potential
                for j in range(i+1,len(group)):
                    testTask = group[j][0]
                    if testTask != currentTask:
                        pot = self.transfer_potential(testTask,currentTask,graph.list_children(currentTask))
                        if pot > maxTransferPot:
                            maxTransferPot = pot
                            argMaxTask = testTask
                if argMaxTask != None and maxTransferPot > self.thresholdTask:
                    # Add connection
                    connections.append((argMaxTask,currentTask))
                    graph.add(argMaxTask,currentTask)
        
        #Creating graph structure
        graph = Graph(connections,directed = True)
                 
        #Inter-group transfer
        for i in range(len(groups.keys())):
            #Get task features of the first task in the group 
            #(this is equal for all tasks inside it)
            g = groups[groups.keys()[i]]
            gFeatures = g[0][0].task_features()
            #For all groups that have task features contained in g:
            for j in range(i+1,len(groups.keys())):
                gPrime = groups[groups.keys()[j]]
                gPrimeFeatures = gPrime[0][0].task_features()
                
                if task.is_contained(gPrimeFeatures,gFeatures):
                    #For all tasks in g, get the argmax task in gPrime and
                    #Possibly creates a link
                    for gTaskAndPot in g:
                        gTask = gTaskAndPot[0]
                        maxTransferPot = -float('inf')
                        argMaxTask = None
                        for gPrimeTaskAndPot in gPrime:
                            gPrimeTask = gPrimeTaskAndPot[0]
                            pot = self.transfer_potential(gPrimeTask,gTask,graph.list_children(currentTask))
                            if pot > maxTransferPot:
                                maxTransferPot = pot
                                argMaxTask = gPrimeTask
                        if argMaxTask != None and maxTransferPot > self.thresholdTask:
                             # Add connection
                             graph.add(argMaxTask,gTask)
                             connections.append((argMaxTask,gTask))
                             
        #For all tasks with zero degree, a connection to the target-task is added
        for currentTask in setOfTasks:
            if graph.out_degree(currentTask) == 0:
                graph.add(currentTask,target_task)
        #The task graph is completed
        self.taskGraph = graph
                        
                    
    def transfer_potential(self,currentTask,target_task,curriculumTasks):
        return task.transfer_potential(currentTask,target_task)
    def previous_tasks(self,task):
        """Returns the children for a given task"""
        #if task.name=="target":
        #    return self.taskGraph.all_nodes()
        return self.taskGraph.list_children(task)
                
        
        
        
    def print_result(self):
        """Prints the CUrriculum"""
        print self.taskGraph
           
                
            