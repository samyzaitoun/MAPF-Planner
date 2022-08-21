# MAPF-Planner
 The implementation of the Planner component in a MAPF project conducted at Technion by Prof. Oren Salzman & under the supervision of CRL lab manager David Dovrat.

In this project, we were assigned to implement a ROS component which would take feed published by mocap (OptiTrack) on /tf and create a path for the agents based on the current state of the system. (Agent == Robot | Turtlebot | etc.)
In order to achieve this, we've implemented two major components: Planner & Manager
## Planner Component 
The planner provides a planning service, under the interface of an Action (we'll get to that later).
Given a list of unassigned agent IDs, unassigned goal positions and an assigned agent-to-goal list, it does the following:
* Lookup the transformations of all objects published on /tf (while organizing them into different lists)
* Assign the remaining goals to the remaining agents
* Discretize the transformations into a 2D representation (to be sent as input to a MAPF Solver)
* Solve the 2D array
* Translate 2D Paths to the appropriate transformations for the agents
* Return paths, assigned agents and error message as output

## Manager Component
The manager component provides a support center for agents. Whenever an agent has encountered an event, such as: Finishing it's path / Failing / Being Idle / Etc., it reports it to the Manager, and the Manager decides what to do with this knowledge given the current state of the system.
Given an agent ID and error message, it does the following:
* Call the relevant handling method in accordance with the error message (Which would usually lead to a plan request)
* Create an async plan request from the Planner
* Create a callback for the async future
* When callback is triggered & response is valid:
* * Update the dataset according to the agents that were assigned
* * Publish the paths that were returned to the agents

## Why Action and not Service
Since the Manager acts as an async component which constantly recieves agent complaints, we'll most likely be calling the planner a lot of times consecutively. That being said, it would be a waste to have multiple threads executing near-identical plan requests, knowing that most of them are going to be dumped away. If Planner was a service, there would be nothing to do about this, as we cannot cancel exeucted callbacks. But since we're using an ActionServer instead, we can turn on a "cancel-flag" in the executed callback. Given this ability, we can now check for that flag and stop mid-execution in the case that it's turned on, saving CPU & system cluttering by doing so.

<img src="/diagrams.jpg" align="center" width="800" alt="Project icon">


## Project Dependencies: 
* MAP-Solver By Gil Kaplan: https://gitlab.com/Gil-kapel/mapf-solver
(We're using a slightly alternated version, released at https://gitlab.com/rooren/mapf-solver/ v1.0.0)

 By: Samy Zaitoun, Oren Rosenberg 
