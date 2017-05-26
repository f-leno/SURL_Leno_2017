#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri May 26 14:18:58 2017

Class to store all task parameters (can be built from text files)

@author: Felipe Leno
"""

class Task(object):
    
    #Task Parameters and initial state
    sizeX     = None
    sizeY     = None
    pits      = None
    fires     = None
    treasures = None
    initState = None
    
    def __init__(self, filePath=None):
        """ The source file must be a text file specified as follows:
            <sizeX>;<sizeY>;<objects>
            where <objects> is any number of objects separated with commas and obeying the format:
            <type>:<xPosic>-<yPosic>,<type>:<xPosic>-<yPosic>
            
        """
        #Read task file
        with open(filePath, 'r') as content_file:
            content = content_file.read()
            
        #get size the grid size
        sep = content.split(';')
        self.sizeX = int(sep[0])
        self.sizeY = int(sep[1])
        
        self.initState = sep[2]
        
        #Extracts the number of objects of each type.
        self.treasures = self.initState.count('treasure')
        self.pits = self.initState.count('pit')
        self.fires = self.initState.count('fire')
        
    def num_pits(self):
        return self.pits
        
    def num_fires(self):
        return self.fires
        
    def get_sizeX(self):
        return self.sizeX
        
    def get_sizeY(self):
        return self.sizeY
        
    def num_treasures(self):
        return self.treasures
    def init_state(self):
        return self.initState
        
        
        

