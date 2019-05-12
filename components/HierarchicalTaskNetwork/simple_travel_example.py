"""
The "travel from home to the park" example from my lectures.
Author: Dana Nau <nau@cs.umd.edu>, May 31, 2013
This file should work correctly in both Python 2.7 and Python 3.2.
Repository: https://bitbucket.org/dananau/pyhop
"""

import pyhop


# Helper methods

def taxi_rate(dist):
    return 1.5 + 0.5 * dist


# Operators (primitive tasks)

def walk(state, a, x, y):
    if state.loc[a] == x:
        state.loc[a] = y
        return state
    else:
        return False


def cycle(state, a, x, y):
    if state.loc[a] == x:
        state.loc[a] = y
        return state
    else:
        return False


def fly(state, a, x, y):
    if state.loc[a] == x:
        state.loc[a] = y
        return state
    else:
        return False


def call_taxi(state, a, x):
    state.loc['taxi'] = x
    return state


def ride_taxi(state, a, x, y):
    if state.loc['taxi'] == x and state.loc[a] == x:
        state.loc['taxi'] = y
        state.loc[a] = y
        state.owe[a] = taxi_rate(state.dist[x][y])
        return state
    else:
        return False


def pay_driver(state, a):
    if state.cash[a] >= state.owe[a]:
        state.cash[a] = state.cash[a] - state.owe[a]
        state.owe[a] = 0
        return state
    else:
        return False


pyhop.declare_operators(walk, cycle, call_taxi, ride_taxi, pay_driver, fly)
print('')
pyhop.print_operators()


# Methods (compound tasks)

def travel_by_foot(state, a, x, y):
    if state.dist[x][y] <= 2:
        return [('walk', a, x, y)]
    return False


def travel_by_bicycle(state, a, x, y):
    if not state.weather['raining'] and state.dist[x][y] < 8:
        return [('cycle', a, x, y)]
    return False


def travel_by_taxi(state, a, x, y):
    if state.cash[a] >= taxi_rate(state.dist[x][y]):
        return [('call_taxi', a, x), ('ride_taxi', a, x, y), ('pay_driver', a)]
    return False


def travel_by_flying(state, a, x, y):
    if state.superhero['superman']:
        return [('fly', a, x, y)]
    return False


pyhop.declare_methods('travel', travel_by_flying, travel_by_foot, travel_by_bicycle, travel_by_taxi)

print('')
pyhop.print_methods()

state1 = pyhop.State('state1')
state1.loc = {'me': 'home'}
state1.cash = {'me': 20}
# state1.cash = {'me': 1}
state1.weather = {'raining': False}
# state1.superhero = {'superman': True}
state1.superhero = {'superman': True}
# state1.weather = {'raining': True}
state1.owe = {'me': 0}
# state1.dist = {'home': {'park': 8}, 'park': {'home': 8}}
state1.dist = {'home': {'park': 2}, 'park': {'home': 2}}
# state1.dist = {'home': {'park': 5}, 'park': {'home': 5}}

print("""
********************************************************************************
Call pyhop.pyhop(state1,[('travel','me','home','park')]) with different verbosity levels
********************************************************************************
""")

# print("- If verbose=0 (the default), Pyhop returns the solution but prints nothing.\n")
# pyhop.pyhop(state1, [('travel', 'me', 'home', 'park')])

# print('- If verbose=1, Pyhop prints the problem and solution, and returns the solution:')
# pyhop.pyhop(state1, [('travel', 'me', 'home', 'park')], verbose=1, all_plans=True)

# print('- If verbose=2, Pyhop also prints a note at each recursive call:')
# pyhop.pyhop(state1, [('travel', 'me', 'home', 'park')], verbose=2)

print('- If verbose=3, Pyhop also prints the intermediate states:')
pyhop.pyhop(state1, [('travel', 'me', 'home', 'park')], verbose=3)


# TODO: Terms:
# TODO: Operators: parameterized descriptions of what the basic actions do
# TODO: Actions: operators with arguments
# TODO: Method: parameterized description of a possible way to perform a compound task by performing a collection of subtasks
# TODO: There may be more than one method for the same task
