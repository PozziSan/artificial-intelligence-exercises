from search_exercises.graph import Node, UndirectedGraph, GraphProblem
from search_exercises.utils import PriorityQueue, memoize
from search_exercises import exercises_map_dict, exercises_map_coordinates_dict

exercises_map = UndirectedGraph(exercises_map_dict)
exercises_map.locations = exercises_map_coordinates_dict
exercises_problem = GraphProblem('I', 'F', exercises_map)


def best_first_graph_search(problem, f):
    iterations = 0

    f = memoize(f, 'f')
    node = Node(problem.initial)

    iterations += 1

    if problem.goal_test(node.state):
        iterations += 1
        return iterations, node

    frontier = PriorityQueue('min', f)
    frontier.append(node)

    iterations += 1

    explored = set()

    while frontier:
        node = frontier.pop()

        iterations += 1

        if problem.goal_test(node.state):
            iterations += 1

            return iterations, node

        explored.add(node.state)

        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)

                iterations += 1
            elif child in frontier:
                incumbent = frontier[child]

                if f(child) < f(incumbent):
                    del frontier[incumbent]

                    frontier.append(child)

                    iterations += 1

        iterations += 1

    return None


def greedy_best_first_search(problem, h=None):
    h = memoize(h or problem.h, 'h')

    return best_first_graph_search(problem, lambda n: h(n))


def uniform_cost_search_graph(problem):
    return best_first_graph_search(problem, lambda node: node.path_cost)


def a_star_search_graph(problem, h=None):
    h = memoize(h or problem.h, 'h')

    return best_first_graph_search(problem, lambda n: n.path_cost + h(n))


def hill_climb(problem, max_iterations):
    iterations = 0

    node = Node(problem.initial)

    objective_function = lambda n: - n.path_cost
    f = memoize(objective_function)

    iterations += 1

    #check if initial is final
    if problem.goal_test(node.state):
        iterations += 1
        return iterations, node

    frontier = PriorityQueue('min', f)
    frontier.append(node)

    explored = set()

    while iterations <= max_iterations:
        node = frontier.pop()

        iterations += 1

        if problem.goal_test(node.state):
            iterations += 1

            return iterations, node

        explored.add(node.state)

        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)

                iterations += 1
            elif child in frontier:
                incumbent = frontier[child]

                if f(child) < f(incumbent):
                    del frontier[incumbent]

                    frontier.append(child)

                    iterations += 1

        iterations += 1

    return iterations, node


if __name__ == '__main__':
    hill_climb_max_iterations = 40

    greedy_iteration, greedy_best_node = greedy_best_first_search(exercises_problem)
    uniform_cost_iteration, uniform_cost_best_node = uniform_cost_search_graph(exercises_problem)
    a_star_iteration, a_star_best_node = a_star_search_graph(exercises_problem)
    hill_climb_iteration, hill_climb_best_node = hill_climb(exercises_problem, hill_climb_max_iterations)

    print(f"""
"RESULTS:

Greedy Iteration First Search: 

number of iterations: {greedy_iteration}
best path: {greedy_best_node.solution()}

---------------------------------------

Uniform Cost Search

number of iterations: {uniform_cost_iteration}
best path: {uniform_cost_best_node.solution()}

---------------------------------------

A* 

number_of_iterations: {a_star_iteration}
best_path: {a_star_best_node.solution()}

---------------------------------------

Hill Climb

number of iterations: {hill_climb_iteration} {' (Max)' if hill_climb_iteration >= hill_climb_max_iterations else ''}
best_path: {hill_climb_best_node.solution()}
""")

