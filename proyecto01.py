# -*- coding: utf-8 -*-


#Carlos Edgardo López Barrera
#21666


import pygame
import numpy as np
from collections import deque, defaultdict
import heapq
import sys
import time 


#DataStructures

class priority:
    def __init__(self, items, comparator):
        self.items = items
        self.comparator = comparator
    def empty(self):
        return  len(self.items) == 0
    def first(self):
        return  self.items[0]
    def remove_first(self):
        if not self.empty():
            return self.items.pop(0)
        return None
    def insert(self, newElement):
        if not self.empty():
            index = 0
            itemsLength = len(self.items)
            while index<itemsLength and self.comparator(newElement, self.items[index]):
                index+=1
            if index<itemsLength:
                self.items.insert(index, newElement)  
            elif index==len(self.items):
                self.items.append(newElement)
        else:
            self.items.append(newElement)
        return self.items

class lifo:
    def __init__(self, items):
        self.items = items
    def empty(self):
        return  len(self.items) == 0
    def first(self):
        return  self.items[len(self.items)-1]
    def remove_first(self):
        if not self.empty():
            return self.items.pop(len(self.items)-1)
        return None
    def insert(self, newElement):
        self.items.append(newElement)
        return self.items
        
class fifo:
    def __init__(self, items):
        self.items = items
    def empty(self):
        return  len(self.items) == 0
    def first(self):
        return  self.items[0]
    def remove_first(self):
        if not self.empty():
            return self.items.pop(0)
        return None
    def insert(self, newElement):
        self.items.append(newElement)
        return self.items





#BFS
def BreadthFirstSearch(graph, init_node, goal_node, screen, colors, cell_size):
    queue = fifo([(init_node, [init_node])]) 
    visited = set([init_node])
    iterations = 0 

    while not queue.empty():
        iterations += 1  
        current_node, path = queue.remove_first()
        visited.add(current_node)

        draw_current_node(current_node, screen, colors, cell_size)
        pygame.display.flip()
        pygame.time.delay(10) 

        if current_node == goal_node:
            draw_path(path, screen, colors, cell_size)
            pygame.display.flip()
            print("Numero de iteraciones:", iterations) 
            return path

        for node in graph[current_node]:
            if node not in visited:
                queue.insert((node, path + [node]))

    print("Numero total de iteraciones:", iterations)  
    

#  DFS
def DepthFirstSearch(graph, init_node, goal_node, screen, colors, cell_size):
    stack = lifo([(init_node, [init_node])]) 
    visited = set()
    iterations = 0 

    while not stack.empty():
        iterations += 1  
        current_node, path = stack.remove_first()
        if current_node not in visited:
            visited.add(current_node)

            draw_current_node(current_node, screen, colors, cell_size)
            pygame.display.flip()
            pygame.time.delay(10) 

            if current_node == goal_node:
                draw_path(path, screen, colors, cell_size)  
                pygame.display.flip()
                print("Numero de iteraciones:", iterations) 
                return path

            neighbours = graph.get(current_node, [])
            for node in neighbours:
                if node not in visited:
                    new_path = path + [node]
                    stack.insert((node, new_path))




#  DLS
def DepthDelimitedSearch(graph, startNode, endNode, limit, screen, colors, cell_size):
    stack = lifo([(startNode, [startNode])]) 
    visited = set()
    predecessors = {startNode: None}
    iterations = 0 
    count = 0
    
    while not stack.empty() and limit >= 0:
        iterations += 1
        count += 1
        current_node, path = stack.remove_first()
        draw_current_node(current_node, screen, colors, cell_size) 
        pygame.display.flip()
        pygame.time.delay(10)

        if current_node == endNode:
            print("Iteraciones DLS: " + str(count))
            draw_path(path, screen, colors, cell_size) 
            pygame.display.flip()
            print("Numero de iteraciones:", iterations) 
            return path
        
        visited.add(current_node)

        for n in reversed(graph[current_node]):
            if n not in visited:
                new_path = path + [n]
                stack.insert((n, new_path))
                predecessors[n] = current_node

        limit -= 1  

    return None




# GBFS
def GreedyBestFirstSearch(comparator, graph, init_node, goal_node, heuristicFunction, screen, colors, cell_size):
    iteration = 0
    queue = priority([(init_node, 0, [init_node], heuristicFunction[init_node])], comparator)
    visited = set()  

    while not queue.empty():
        iteration += 1
        current_node, current_cost, current_path, _ = queue.remove_first()

        if current_node not in visited:
            visited.add(current_node)
            draw_current_node(current_node, screen, colors, cell_size)
            pygame.display.flip()
            pygame.time.delay(10) 

            if current_node == goal_node:
                draw_path(current_path, screen, colors, cell_size) 
                pygame.display.flip()
                print("->".join(current_path) + "->" + str(current_cost))
                print("\nIteraciones en el while:", iteration)
                return current_path

            children = graph.get(current_node, [])
            for child in children:
                if child[0] not in visited:
                    new_path = current_path + [child[0]]
                    new_cost = current_cost + child[1]
                    queue.insert((child[0], new_cost, new_path, heuristicFunction[child[0]]))



# A*
def AStarSearch(comparator, graph: dict, init_node: str, goal_node: str, heuristicFunction: dict):
    iteration = 0
    queue = priority([], comparator)
    visited = set() 
    queue.insert((init_node, 0, [init_node], heuristicFunction[init_node] + 0))  
    path = None

    while not queue.empty():
        iteration += 1
        current_node, current_cost, current_path, _ = queue.remove_first()

        if current_node not in visited:
            visited.add(current_node)
            if current_node == goal_node:
                print("->".join(current_path) + "->" + str(current_cost))
                path = current_path
                break
            children = graph.get(current_node, [])
            for child in children:
                if child[0] not in visited:
                    new_path = current_path + [child[0]]
                    g_value = current_cost + child[1]  
                    f_value = g_value + heuristicFunction[child[0]] 
                    queue.insert((child[0], g_value, new_path, f_value))

    print("\nIterations:", iteration)
    return path


#Se va dibujando paso a paso
def draw_current_node(node, screen, colors, cell_size):
    x, y = node
    color = colors[4]
    pygame.draw.rect(screen, color, (x*cell_size, y*cell_size, cell_size, cell_size))

def draw_path(path, screen, colors, cell_size):
    for node in path:
        x, y = node
        color = colors[4]
        pygame.draw.rect(screen, color, (x*cell_size, y*cell_size, cell_size, cell_size))



#Se lee el laberinto
def read_maze_from_file(file_path):
    with open(file_path, 'r') as file:
        maze = []
        start_x = start_y = end_x = end_y = None
        for y, line in enumerate(file):
            row = []
            for x, char in enumerate(line.strip()):
                row.append(char)
                if char == '2':
                    start_x, start_y = x, y
                elif char == '3':
                    end_x, end_y = x, y
            maze.append(row)
    return maze, (start_x, start_y), (end_x, end_y)



#Se pasa a un grafo
def convert_maze_to_graph(maze):
    graph = {}
    for y, row in enumerate(maze):
        for x, cell in enumerate(row):
            if cell != '0':  
                graph[(x, y)] = []
                if y > 0 and maze[y-1][x] != '0':
                    graph[(x, y)].append((x, y-1))
                if y < len(maze)-1 and maze[y+1][x] != '0':  
                    graph[(x, y)].append((x, y+1))
                if x > 0 and maze[y][x-1] != '0': 
                    graph[(x, y)].append((x-1, y))
                if x < len(row)-1 and maze[y][x+1] != '0':  
                    graph[(x, y)].append((x+1, y))
    return graph



#Dibujar labaenrinto
def draw_maze(maze, screen, colors, cell_size):
    for y, row in enumerate(maze):
        for x, cell in enumerate(row):
            color = colors[int(cell)]
            pygame.draw.rect(screen, color, (x*cell_size, y*cell_size, cell_size, cell_size))
            


#Ejecución principal
def main(maze_file):
    start_time = time.time()  # Inicia el contador de tiempo
    
    pygame.init()
    cell_size = 10
    colors = {0: (52, 73, 94), 1: (177, 177, 177), 2: (0, 255, 0), 3: (255, 0, 0), 4: (0, 0, 255), 5: (255, 0, 0)}

    maze, (start_x, start_y), (end_x, end_y) = read_maze_from_file(maze_file)
    graph = convert_maze_to_graph(maze)
    init_node = (start_x, start_y) 
    goal_node = (end_x, end_y) 

    screen_size = (len(maze[0]) * cell_size, len(maze) * cell_size)
    screen = pygame.display.set_mode(screen_size)
    pygame.display.set_caption("Laberinto")

    draw_maze(maze, screen, colors, cell_size)
    pygame.display.flip()

    path = BreadthFirstSearch(graph, init_node, goal_node, screen, colors, cell_size)
    path = DepthFirstSearch(graph, init_node, goal_node, screen, colors, cell_size)
    path = DepthDelimitedSearch(graph, init_node, goal_node, 2000, screen, colors, cell_size)
    path = GreedyBestFirstSearch(graph, init_node, goal_node, screen, colors, cell_size)
    path = AStarSearch(graph, init_node, goal_node, screen, colors, cell_size)
    
    for node in path:
        x, y = node
        pygame.draw.rect(screen, (255, 0, 0), (x*cell_size, y*cell_size, cell_size, cell_size))
        pygame.display.flip()

    end_time = time.time()  # Finaliza el contador de tiempo
    execution_time = end_time - start_time  # Calcula el tiempo total de ejecución
    print(f"Tiempo de ejecucion: {execution_time} segundos")  # Imprime el tiempo de ejecución

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

if __name__ == "__main__":
    maze_file = "Prueba_1.txt" 
    main(maze_file)