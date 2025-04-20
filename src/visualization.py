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

    def create_interactive_map(self, pathDict, savePath=None):

        # Function creates an interactive Folio map.

        # Checks to ensure that the PathVisualizer has a set graph.

        if self.graph is None:

            raise ValueError("Graph not set.")

        # Compute map center by averaging input latitude and longitudes.

        latitude = np.mean([data['y']
                            for _, data in self.graph.nodes(data=True)])
        longitude = np.mean([data['x']
                             for _, data in self.graph.nodes(data=True)])

        # Create Folio map and set visualizer settings.

        m = folium.Map(location=[latitude, longitude], zoom_start=13)
        colors = {'dijkstra': 'red', 'gbfs': 'blue'}

        # Iterate through path dict and add path to map.

        for algoName, path in pathDict.items():

            if path is not None and len(path) >= 2:

                coords = [(self.graph.nodes[node]['y'],
                           self.graph.nodes[node]['x']) for node in path]

            # Create polyline using path coordinates.

            folium.PolyLine(coords, color=colors.get(
                algoName, 'green'), weight=4, opacity=0.8, tooltip=f"{algoName} path").add_to(m)

            # Add start and end markers.

            folium.Marker(location=[coords[0][0], coords[0][1]], popup='Start', icon=folium.Icon(
                color='green')).add_to(m)
            folium.Marker(location=[coords[-1][0], coords[-1][1]],
                          popup='End', icon=folium.Icon(color='red')).add_to(m)

            # Saves the map to the provided path if possible.

            if savePath:

                os.makedirs(os.path.dirname(savePath), exist_ok=True)

        m.save(savePath)
        print(f"Interactive map saved to {savePath}")
        return m

    def visualize_performance_comparison(self, metrics, savePath=None):

        # Visualizes a bar chart for the algorithm performance metric comparison.

        # Sets bar figures using input metrics.

        dijkstraTime = metrics['dijkstra']['time']
        gbfsTime = metrics['gbfs']['time']
        dijkstraDistance = metrics['dijkstra']['distance']
        gbfsDistance = metrics['gbfs']['distance']

        # Create plot.

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

        # Plot algorithm comparisons.

        algorithms = ['Dijkstra', 'GBFS']
        times = [dijkstraTime, gbfsTime]
        ax1.bar(algorithms, times, color=['red', 'blue'])
        ax1.set_title('Processing Time (seconds)')
        ax1.set_ylabel('Time (s)')

        # Iterate through times, adding them to the top of the bars in the first chart.

        for i, v in enumerate(times):

            ax1.text(i, v + 0.001, f"{v:.6f}s", ha='center')

            # Compare and plot differences in path length.

            distances = [dijkstraDistance, gbfsDistance]
            ax2.bar(algorithms, distances, color=['red', 'blue'])
            ax2.set_title('Path Distance (meters)')
            ax2.set_ylabel('Distance (m)')

        # Iterate through distances, adding them to the top of the bars in the second chart.

        for i, v in enumerate(distances):

            ax2.text(i, v + 5, f"{v:.2f}m", ha='center')

            plt.tight_layout()

        if savePath:

            # Save algorithm comparison to the provided path if possible.

            os.makedirs(os.path.dirname(savePath), exist_ok=True)
            plt.savefig(savePath, dpi=300, bbox_inches='tight')
            plt.close()
            print(f"Performance comparison saved to {savePath}")
        else:

            # Otherwise display the plot.

            plt.show()
