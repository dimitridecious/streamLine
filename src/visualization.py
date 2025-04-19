import matplotlib.pyplot as plt
import folium
import numpy as np
import os


class PathVisualizer:
    def __init__(self, graph=None):
        # init with a graph
        self.graph = graph

    def set_graph(self, graph):
        # set graph
        self.graph = graph
