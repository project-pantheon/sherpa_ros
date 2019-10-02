# -*- coding: utf-8 -*-
"""
Created on Sat Sep 22 19:20:08 2018

@author: Alessandro
"""
from __future__ import division
import numpy
import matplotlib.pyplot as plt

from concorde.tsp import TSPSolver
from numpy import linalg as LA

molt = 1000

def readFile(name,n_nodes):
    #edges = numpy.zeros(shape=(n_nodes, 2),dtype=int)
    nodes = numpy.zeros(shape=(n_nodes, 1),dtype=int)
    file = open(name,"r")
    i=0
    for line in file:
        elem = line.split(" ")
        for e in elem:
            if e!="\n":
                nodes[i,0]=int(e)
                i+=1
         
    '''for i in range(len(nodes)-1):
        edges[i,:] = [nodes[i],nodes[i+1]]
     
    edges[i+1,:] = [nodes[i+1],nodes[0]]'''
    return nodes    
        

def matlab(file):
    ed = numpy.zeros(shape=(400,2),dtype=int)
    
    file = open(file,"r")
    i=0
    for line in file:
        ed[i][0]=line.split(" ")[0]
        ed[i][1]=line.split(" ")[1]
        i+=1
    
    return ed

def getNodes(n):
    nodes = []
    for i in range(n):
        nodes.append(str(i))
    return nodes

def mapPoint(nodes,colors):
    
    dic = {}
    for n in nodes:
        col = colors.get(n)  
        for i in range(4):
            dic[(int(n)*4+i)]=col
            
    return dic       

def generateRandomCenters(num_rows, num_col):
    
    n = num_rows*num_col
    centers = numpy.zeros(shape=(n, 2))
    centers[:,0]=20000 * numpy.random.rand(n)
    centers[:,1]=20000 * numpy.random.rand(n)
    
    
    return centers    

def generateCenters(num_rows, num_col, dist_row,dist_col):
     
    xx =1*molt
    yy =1*molt
    centers = numpy.zeros(shape=(num_rows*num_col, 2))
    h =0;
    for i in range(0,num_rows):  
        for j in range(0,num_col):
            
            centers[h,0]=xx
            centers[h,1]=yy
            h=h+1
            yy =yy +dist_col
            
        xx =xx +dist_row
        yy = 1*molt
        
    return centers        
            
    
def generateRefFrame(centers,r):
    
    h=0
    #1.15176*molt
    print(" raggio = "+ str(r))
    v = numpy.matrix([[0,r],[r,0],[-r,0],[0,-r]])
    points =numpy.zeros(shape =(4*centers.shape[0],2))
    for i in range(centers.shape[0]):
        points[h:(h+4),:]=v+centers[i,:]
        h=h+4
       
    return points        
            
def generateFile(namefile,num_tree,points):
    
    f = open(namefile,"w")
    f.write("NAME: trees\nTYPE:TSP\nCOMMENT: waypoints\nDIMENSION: %d\nEDGE_WEIGHT_TYPE: EUC_2D\nNODE_COORD_SECTION\n" % (num_tree*4))
    
    
    for i in range(num_tree*4):
        s = str(i+1) + " "+ str(points[i][0])+" "+ str(points[i][1])+ "\n"
        f.write(s)
        
    f.write("EOF")
    f.close()
    
def formatEdges(solution):
    
    edges = numpy.zeros(shape=(solution.size, 2),dtype=int)    
    for i in range(solution.size-1):
        edges[i,:]=[solution[i],solution[i+1]]
        
    edges[i+1,:]=[solution[i+1], solution[0]]    
    return edges
      
def groupByColour(dic):
    dic2 ={}
    
    for key in dic:
        val = dic.get(key)
        if val in dic2:
            elem = dic2[val]
            elem.append(key)
            dic2[val]= elem
        else:
            elem = []
            elem.append(key)
            dic2[val]=elem
    return dic2
def get_Trees_Edges(neighbors,n_trees):
    
    n_edges_i = []
    n_edges_j = []
    h=0
    for key in neighbors:
        elem = neighbors.get(key)
        for i in range(len(elem)):
            n_edges_i.append(int(key))
            n_edges_j.append(int(elem[i]))
            h+=1
    return n_edges_i,n_edges_j

def plotColouredGraph(centers,neighbors,colors,n_trees):
    x = centers[:,0].flatten()
    y = centers[:,1].flatten()
    
    dic = groupByColour(colors)
    reds = dic.get('R')
    greens = dic.get('G') 
    blues =dic.get('B')
    #yellows = dic.get('Y')
    
    plt.figure(figsize=(8,6))
    for i in range(len(reds)):
        plt.plot(centers[int(reds[i]),0],centers[int(reds[i]),1],'o',color ='r',markersize=15)
        
    for i in range(len(greens)):
        plt.plot(centers[int(greens[i]),0],centers[int(greens[i]),1],'o',color ='g',markersize=15)
        
    for i in range(len(blues)):
        plt.plot(centers[int(blues[i]),0],centers[int(blues[i]),1],'o',color ='b',markersize=15)
    
    '''for i in range(len(yellows)):
        plt.plot(centers[int(yellows[i]),0],centers[int(yellows[i]),1],'o',color ='y',markersize=2)'''
    
    e1,e2 = get_Trees_Edges(neighbors,n_trees)
    e_x = [x[e1],x[e2]]
    e_y = [y[e1],y[e2]]
    plt.plot(e_x, e_y, linestyle='-', color='y')  
 
def plotPoints(points,edges):
    x = points[:,0].flatten()
    y = points[:,1].flatten()
    fig, ax= plt.subplots(figsize=(10,8))
    #ax.scatter(points[:,0],points[:,1])
    
    num =range(points.shape[0])  
    
    for i,txt in enumerate(num):
        ax.annotate(txt, (points[i,0], points[i,1]),)
    
    plt.plot(x[edges.T], y[edges.T], linestyle='-', color='y')     

def plotWithoutLines(points,centers,n_col,n_row):
    x = points[:,0].flatten()
    y = points[:,1].flatten()
    plt.figure(figsize=(16,12))
    plt.plot(centers[:,0],centers[:,1],'o',color ='g')
    plt.plot(x,y,'o',color ='r')
    name = "points "+str(n_col)+"x"+str(n_row)
    plt.savefig("/Users/Alessandro/Desktop/ProvaConc/"+name, dpi = 800 ,format = 'pdf')
    

def plotGraph(points,edges,centers,n_col,n_row):
    
    x = points[:,0].flatten()
    y = points[:,1].flatten()

    plt.figure(figsize=(12,8))
    plt.plot(centers[:,0],centers[:,1],'o',color ='g')
    plt.plot(x[edges.T], y[edges.T], linestyle='-', color='y',markerfacecolor='red', marker='o') 
    #plt.plot(x,y,'o',color ='r')
    #plt.plot(x[edges.T], y[edges.T], color='y',markerfacecolor='red', marker='o') 
    
    name = str(n_col)+"x"+str(n_row)
    #plt.savefig("/Users/Alessandro/Desktop/ProvaConc/"+name, format = 'pdf')
    '''
    fig, ax= plt.subplots(figsize=(12,10))
    ax.scatter(centers[:,0],centers[:,1])
      
    num =[]    
    for i in range(thetas.shape[1]):
        num.append(round(((thetas[0][i]+numpy.pi/2)%(numpy.pi/2))*(180/(numpy.pi)),2))
    
    for i,txt in enumerate(num):
        ax.annotate(txt, (centers[i,0], centers[i,1]))
        
        
    plt.figure(figsize=(10,8))'''
    
    #plt.hist(num, bins='auto')  # arguments are passed to np.histogram
    #plt.title("Histogram with 'auto' bins")
    plt.show()
    
def plotGraphLight(points,edges,centers):
    
    x = points[:,0].flatten()
    y = points[:,1].flatten()

    plt.figure(figsize=(10,8))
    plt.plot(centers[:,0],centers[:,1],'o',color ='g')
    plt.plot(x[edges.T], y[edges.T], linestyle='-', color='y',markerfacecolor='red', marker='o') 
 

def rotMatrix(x,y,theta,cx,cy):
    p = numpy.zeros(shape=(1, 2))
    new_x = x-cx;
    new_y = y-cy;

    p[0][0]= new_x*numpy.cos(theta)-new_y*numpy.sin(theta)+ cx
    p[0][1]= new_x*numpy.sin(theta)+new_y*numpy.cos(theta)+ cy
    
    return p
    
def computePoints(thetas, points_ref, centers):
    
    points = numpy.zeros(shape=(points_ref.shape[0], 2))
    
    for i in range(points_ref.shape[0]):
        h = i//4
        points[i,:] = rotMatrix(points_ref[i,0],points_ref[i,1],thetas[0][h],centers[h,0],centers[h,1])
    
    #points = insertDummy(points)
    return points

def insertDummy(points):
    n_points = numpy.zeros(shape=(1+points.shape[0], 2))
    n_points[0][:] = [-1,-1]
    for i in range(points.shape[0]):
        n_points[i+1][:]=points[i][:]
    return n_points
    
'''def computeGradient(points_ref,centers,thetas,funz_ob,num_trees):
    
    
    epsilon = 0.00001
    deltas = numpy.eye(num_trees)*epsilon
    gradient = numpy.zeros(shape=(1,num_trees))
    
    for i in range(num_trees):
        thetas_delta = thetas +deltas[i,:]
        points = computePoints(thetas_delta,points_ref, centers)
        funz_ob_delta = computeTsp(num_trees,points)[0]
        gradient[0][i]=(funz_ob_delta-funz_ob)/epsilon
        
    return gradient'''
        


def armijo(direction, fo, centers, thetas, points_ref,n_trees):
    
    alpha =0.01
    gamma = 0.25
    iter = 0
    condition = False

    while iter<50 and (not condition):

        new_thetas = thetas -alpha*direction
        n_points = computePoints(new_thetas, points_ref, centers)
        f_delta = computeTsp(n_trees,n_points)[0]
        
        if f_delta<= fo+gamma*alpha*numpy.dot(direction,numpy.transpose(-direction)):
            condition = True
            
        else:
            alpha = alpha*gamma
        
        if LA.norm(new_thetas-thetas)<0.000001:
            points = computePoints(thetas, points_ref, centers)
            return thetas,points,fo
       
        iter = iter +1 
        
    fo =f_delta
    
    return new_thetas, n_points, fo

    
    
def computeTsp(n_trees,points):   
    
    generateFile("trees.tsp",n_trees,points)
    solver = TSPSolver.from_tspfile("/Users/Alessandro/Documents/Pyworkspace/trees.tsp")
    solution = solver.solve()
    sol = solution.tour
    fo =computeFo(sol,points)
    return fo,sol
    
def penalty(thetas):
    cons1 = 0
    cons2 = numpy.pi/2
    pen =0 
    for i in range(thetas.shape[1]):
        err1 = thetas[0][i]-cons1
        err2 = cons2-thetas[0][i]
        if err1<0:
            pen = pen + numpy.power(err1,2)+4
        if err2<0:
            pen = pen +numpy.power(err2,2)+4
            
    return round(pen,4)       

def computeFo(sol,points):
    
    edges = formatEdges(sol)
    x = points[:,0].flatten()
    y = points[:,1].flatten()
    dist = numpy.sqrt(numpy.power(x[edges.T[0]]-x[edges.T[1]],2)+numpy.power(y[edges.T[0]]-y[edges.T[1]],2))
    
    fo =round(numpy.sum(dist)/molt,4)#+penalty(thetas)
    return fo
    
    
    
def writeAngles(thetas):

    f = open("angles.txt","w+")
    f.write("*** Thetas Angles ****\n\n")
    for i in range(thetas.shape[1]):
        num = (thetas[0][i]+2*numpy.pi%(2*numpy.pi))*(360/(2*numpy.pi))
        
        s = str(i+1) + " "+ str(num)+"\n"
        f.write(s)
        
    f.write("EOF")
    f.close()
         
    
'''def gradientConiugate(points_ref,centers,thetas, fo, num_trees):
        
    epsilon = 0.00001;
    gradient_k = computeGradient(points_ref,centers,thetas, fo,num_trees)
       
    if LA.norm(gradient_k) < epsilon:
        return True, thetas, fo
                
    direction_k = -gradient_k    #step 0:  direction = -gradient
    b_fo =fo
    b_thetas = thetas
    i=0
    flag = False
    
    while i<20 and (not flag):
        
       n_thetas,points,fo = armijo(direction_k,fo,centers, thetas, points_ref,num_trees)  # x_k+1 = x_k +alpha*d_k
       gradient_k1 =  computeGradient(points,centers,n_thetas, fo,num_trees)
      
       if LA.norm(gradient_k) < epsilon:
           flag = True    
       else:
           beta = numpy.dot(gradient_k1, numpy.transpose(gradient_k1))/(numpy.dot(gradient_k, numpy.transpose(gradient_k)))
           direction_k1 = -gradient_k1+beta*direction_k
           direction_k = direction_k1
           gradient_k = gradient_k
           
       print("iter #"+ str(i)+":  " +str(fo))   
       if fo<b_fo:
           
           b_fo = fo
           b_thetas =n_thetas
             
           
       i=i+1
       
    return b_thetas,b_fo      
'''     

def gradientExtimation(points_ref,centers,thetas, funz_ob, num_trees,flag):
    
    
    epsilonP = 0.001
    epsilonN = -0.001
    eps = 0.000001
    deltasP = numpy.eye(num_trees)*epsilonP;
    deltasN = numpy.eye(num_trees)*epsilonN;
    gradientP = numpy.zeros(shape=(1,num_trees))
    gradientN = numpy.zeros(shape=(1,num_trees))
    
    for i in range(num_trees):
    
        ''' positive epsilon '''
        thetas_deltaP = thetas + deltasP[i,:]
        pointsP = computePoints(thetas_deltaP, points_ref, centers)
        funz_ob_deltaP = computeTsp(num_trees,pointsP)[0]
        gradientP[0][i] =(funz_ob_deltaP-funz_ob)/epsilonP;
        
    
        '''negative epsilon'''
        thetas_deltaN = thetas +deltasN[i,:]
        pointsN = computePoints(thetas_deltaN, points_ref, centers)
        funz_ob_deltaN = computeTsp(num_trees,pointsN)[0]
        gradientN[0][i] =(funz_ob_deltaN-funz_ob)/epsilonN;
        
    
        
    #if numpy.dot(gradientP,-numpy.transpose(gradientP))>epsilonP*LA.norm(gradientP)*LA.norm(-gradientP) and LA.norm(gradientN)>sigmaN:
     #   flag = True
    
        
    #gradientP = gradientP/LA.norm(gradientP)
    #gradientN = gradientN/LA.norm(gradientN)
    
    new_thetasP,pointsP,new_funz_obP = armijo(gradientP,funz_ob,centers, thetas, points_ref,num_trees)
    new_thetasN,pointsN,new_funz_obN = armijo(gradientN,funz_ob,centers, thetas, points_ref,num_trees)
    
    if new_funz_obP < new_funz_obN: 
        
        new_funz_ob = new_funz_obP
        new_thetas = new_thetasP
        nG =LA.norm(gradientP)
      
    else:
            
        new_funz_ob = new_funz_obN
        new_thetas = new_thetasN
        nG =LA.norm(gradientN)
        
    '''stop criterion'''
    
   
    nThetas = LA.norm(new_thetas-thetas)
    maxTheta = max(LA.norm(thetas),1)
    nFo = numpy.abs(new_funz_ob-funz_ob)
    maxFo = max(numpy.abs(funz_ob),1)
    
    
    if (nG< epsilonP) or ((nThetas/maxTheta)<= eps) or ((nFo/maxFo)<=eps):
        return True, thetas, funz_ob


     
     
    #print(str(LA.norm(gradientP))+ "  "+str(LA.norm(gradientN)))
  
     
    return flag,new_thetas,new_funz_ob 

     
    
def tree_Neighbors(edges):
    neighbors = {}
    edges_tree = edges//4
    for e in edges_tree:
            elem1 = neighbors.get(str(e[0]))
            elem2 = neighbors.get(str(e[1]))
            
                
            if not elem1 and e[1] != e[0]:
                elem1 = []
                elem1.append(str(e[1]))
                neighbors[str(e[0])]=elem1 
            elif elem1 and (str(e[1]) not in elem1) and e[1] != e[0]:
                elem1.append(str(e[1]))
                neighbors[str(e[0])]=elem1  
                
            if not elem2 and e[1] != e[0]: 
                elem2= []
                elem2.append(str(e[0]))
                neighbors[str(e[1])]=elem2 
                    
            elif elem2 and (str(e[0]) not in elem2) and e[1] != e[0]:
                elem2.append(str(e[0]))
                neighbors[str(e[1])]=elem2 
                 
                
       
    return neighbors

#edges = readFile("trees.sol",400)