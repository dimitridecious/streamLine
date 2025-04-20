import os
import sys
import time
import osmnx as ox
import argparse
from math import sqrt, cos, radians
from src.algorithms import PathfindingAlgorithms
from src.visualization import PathVisualizer


def get_route_network(origin_lat, origin_lng, dest_lat, dest_lng, buffer_dist=2000):
    # here get/load a network
    cache_id = f"route_{origin_lat:.4f}_{origin_lng:.4f}_to_{
        dest_lat:.4f}_{dest_lng:.4f}_buff{buffer_dist}"
    cache_path = f"data/{cache_id}.graphml"
    # made for testing/efficiency: check if route cached already
    if os.path.exists(cache_path):
        print("Loading cached route network...")
        G = ox.io.load_graphml(cache_path)
        print(f"Network loaded with {len(G.nodes)
                                     } nodes and {len(G.edges)} edges")
        return G

    # calculate the midpoint
    mid_lat = (origin_lat + dest_lat) / 2
    mid_lng = (origin_lng + dest_lng) / 2

    # calculate radius
    # rough conversion to meters
    lat_diff = abs(origin_lat - dest_lat) * 111000
    lng_diff = abs(origin_lng - dest_lng) * 111000 * cos(radians(mid_lat))
    distance = sqrt(lat_diff**2 + lng_diff**2)

    # bug fix: ensure enough buffer around the route
    dist = max(distance / 2 + buffer_dist, buffer_dist * 2)
    print(f"pulling network around route (buffer: {dist:.0f}m)...")

    try:
        # here, get a network centered at the midpoint with radius slightly larger than half the distance
        start_time = time.time()
        G = ox.graph_from_point(
            (mid_lat, mid_lng), dist=dist, network_type="drive")

        # make sure the network includes our origin and destination points
        # need to verify that there are nodes near our origin and destination
        # if not, increase the buffer and try again
        max_attempts = 3
        attempt = 1
        while attempt <= max_attempts:
            # try to find the nearest nodes to our origin and destination
            try:
                origin_node = ox.distance.nearest_nodes(
                    G, origin_lng, origin_lat)
                dest_node = ox.distance.nearest_nodes(G, dest_lng, dest_lat)
                if origin_node != dest_node:
                    break
                # same node, increase buffer by 50%
                print(
                    "Origin and destination are mapping to the same node. Increasing buffer...")
                dist *= 1.5
                G = ox.graph_from_point(
                    (mid_lat, mid_lng), dist=dist, network_type="drive")
                attempt += 1
            except Exception as e:
                print(f"Error finding nodes (attempt {attempt}): {e}")
                dist *= 1.5
                G = ox.graph_from_point(
                    (mid_lat, mid_lng), dist=dist, network_type="drive")
                attempt += 1

        # bug fix: still no nodes? raise an error
        if attempt > max_attempts:
            raise ValueError(
                "Unable to find distinct nodes for origin and destination after multiple attempts")

        # add edge lengths
        print("Adding edge length attributes...")
        G = ox.distance.add_edge_lengths(G)
        pull_time = time.time() - start_time
        print(f"Network pulled in {pull_time:.2f} seconds")
        print(f"Network has {len(G.nodes)} nodes and {len(G.edges)} edges")

        # cache for future use(testing and demo, or just frequented routes)
        os.makedirs("data", exist_ok=True)
        ox.io.save_graphml(G, cache_path)
        print("Network cached for future use")
        return G
    except Exception as e:
        print(f"Error pulling network: {e}")
        sys.exit(1)


def handle_input():
    # set up CLI parser and prompt user for inputs/args if missing
    parser = argparse.ArgumentParser()
    parser.add_argument('--origin-lat', type=float)
    parser.add_argument('--origin-lng', type=float)
    parser.add_argument('--dest-lat', type=float)
    parser.add_argument('--dest-lng', type=float)
    args = parser.parse_args()

    # bug fix: coords missing? prompt for all again
    if not all([args.origin_lat, args.origin_lng, args.dest_lat, args.dest_lng]):
        print("\nPlease enter the coordinates for origin and destination points.")

        # example usage here
        print("\nExample input/usage:")
        print("  Input - Origin latitude, Origin longitude: 29.9012, -81.3124")
        print("  Input - Destination latitude, Destination longitude: 30.3322, -81.6557")

        # prompts the user, also handle input error
        try:
            origin_raw = input("\nInput - Origin latitude, Origin longitude: ")
            dest_raw = input(
                "\nInput - Destination latitude, Destination longitude: ")
            origin_lat, origin_lon = origin_raw.split(", ")
            origin_lat = float(origin_lat)
            origin_lon = float(origin_lon)
            dest_lat, dest_lon = dest_raw.split(", ")
            dest_lat = float(dest_lat)
            dest_lon = float(dest_lon)
            args.origin_lat = origin_lat
            args.origin_lng = origin_lon
            args.dest_lat = dest_lat
            args.dest_lng = dest_lon
        except ValueError:
            print("ERROR: Coordinates must be valid numbers.")
            sys.exit(1)
    return args


def main():
    # entry point: pull network, compute and compare the paths, then visualize
    print("streamLine - Coordinates must be relatively close together!")
    # get coordinates, too many nodes may cause extreme processing times
    args = handle_input()
    # get the network
    G = get_route_network(args.origin_lat, args.origin_lng,
                          args.dest_lat, args.dest_lng, buffer_dist=5000)
    # find nearest nodes
    print("Finding nearest nodes to coordinates...")
    origin_node = ox.distance.nearest_nodes(
        G, args.origin_lng, args.origin_lat)
    dest_node = ox.distance.nearest_nodes(G, args.dest_lng, args.dest_lat)
    # bug fix: origin and dest node are the same
    if origin_node == dest_node:
        print("ERROR: Origin and destination are mapping to the same node.")
        print("Try using coordinates that are further apart.")
        sys.exit(1)

    print(f"\nOrigin node: {origin_node}")
    print(f"Destination node: {dest_node}")

    # init algorithms and visualizer
    algorithms = PathfindingAlgorithms(G)
    visualizer = PathVisualizer(G)

    # now compare algorithms
    print("\nCalculating paths...")
    stats = algorithms.algorithm_difference(origin_node, dest_node)

    # finally, print results
    print("\nRESULTS")

    print("\nDijkstra's Algorithm:")
    print(f"  Path length: {len(stats['dijkstra']['path'])} nodes")
    print(f"  Distance: {stats['dijkstra']['distance']:.2f} meters")
    print(f"  Computation time: {stats['dijkstra']['time']:.6f} seconds")

    print("\nGreedy Best-First Search:")
    print(f"  Path length: {len(stats['gbfs']['path'])} nodes")
    print(f"  Distance: {stats['gbfs']['distance']:.2f} meters")
    print(f"  Computation time: {stats['gbfs']['time']:.6f} seconds")

    print("\nComparison stats:")
    print(f"  Time difference: {
          stats['comparison']['time_diff']:.6f} seconds")
    print(f"  Time difference percentage: {
          stats['comparison']['time_diff_percent']:.2f}%")
    print(f"  Distance difference: {
          stats['comparison']['distance_diff']:.2f} meters")
    print(f"  Distance difference percentage: {
          stats['comparison']['distance_ratio']:.2f}%")

    # create a directory for visualizations
    os.makedirs('visualizations', exist_ok=True)

    # here generate the 2 visualizations
    print("\nGENERATING VISUALIZATIONS")
    # interactive map
    print("Creating interactive map...")
    visualizer.create_interactive_map(
        {'dijkstra': stats['dijkstra']['path'], 'gbfs': stats['gbfs']['path']}, 'visualizations/interactive_map.html')
    # graph of performance comparison
    print("Creating performance comparison chart...")
    visualizer.visualize_performance_comparison(
        stats, 'visualizations/performance_comparison.png')

    print("\nVISUALIZATION COMPLETE")
    print("Files generated:")
    print("  1. visualizations/interactive_map.html")
    print("  2. visualizations/performance_comparison.png")


if __name__ == "__main__":
    main()
