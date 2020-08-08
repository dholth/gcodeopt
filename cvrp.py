"""Vehicles Routing Problem (VRP)."""

from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import gcodeopt


def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data["distance_matrix"] = []
    for i, from_segment in enumerate(gcodeopt.segments):
        row = []
        data["distance_matrix"].append(row)
        for j, to_segment in enumerate(gcodeopt.segments):
            # distance from i to j
            if i == j:
                row.append(0)
            else:
                row.append(
                    int(gcodeopt.distance(from_segment.endpos, to_segment.startpos))
                )
    data["initial_routes"] = [list(range(1, len(data["distance_matrix"])))]
    data["num_vehicles"] = 1
    data["depot"] = 0
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    indexes = []
    max_route_distance = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = "Route for vehicle {}:\n".format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += " {} -> ".format(manager.IndexToNode(index))
            indexes.append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += "{}\n".format(manager.IndexToNode(index))
        plan_output += "Distance of the route: {}m\n".format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print("Maximum of the route distances: {}m".format(max_route_distance))
    return indexes


def main():
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        60000,  # vehicle maximum travel distance or compute from initial solution
        True,  # start cumul to zero
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # forbid a path and its reversed twin from being followed at the same time
    for i in range(0, len(data["distance_matrix"]), 2):
        routing.AddDisjunction([manager.IndexToNode(i), manager.IndexToNode(i + 1)])

    initial_solution = routing.ReadAssignmentFromRoutes(data["initial_routes"], True)
    # print("Initial solution:")
    # print_solution(data, manager, routing, initial_solution)

    # Set default search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    solution = routing.SolveFromAssignmentWithParameters(
        initial_solution, search_parameters
    )

    # Print solution on console.
    if solution:
        print("Solution after search:")
        solution = print_solution(data, manager, routing, solution)
        print(
            "dropped nodes:",
            set(range(len(data["distance_matrix"]))) - set(solution),
            len(solution),
            "of",
            len(data["distance_matrix"]),
        )
        with open("optimized.gcode", "w+") as optim:
            optim.writelines((str(line) + "\n") for line in gcodeopt.preamble)
            for segment in solution:  # 0 is repeated at start and end
                optim.write("(segment %d)\n" % segment)
                optim.writelines(
                    (str(line) + "\n") for line in gcodeopt.segments[segment].lines
                )


if __name__ == "__main__":
    main()
