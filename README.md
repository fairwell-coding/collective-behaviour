# Collective behaviour - group A

## Public GIT repository 

The public GIT repository can be found under: https://github.com/fairwell-coding/collective-behaviour

## Chosen paper

The project is based on the article: Modeling of Crowd Evacuation With Assailants via a Fuzzy Logic Approach (Zhou, Dong, Wen, Yau and Sun, 2015)

The concept outlined in this original paper establishes the basis for this fuzzy logic simulation.

## Goals

The goal of this scientific project was to reconstruct a crowd simulation of pedestrians trying to evacuate to a common goal while avoiding each other, maneuver around any obstacles in their way and attackers/assailants attempting to catch as many pedestrians as possible before they reach their goal. 

The original paper only outlines the concepts but missed out on an Appendix, detailed and complete definition of membership functions, fuzzy logic rules etc which are reconstructed based on the author's intentions and extended by using appropriate definitions in order to provide a modular, easy to use and extendable simulation. 

## Levels reached

Category I with all its subcategories and Category IV of the paper have been implemented in all their mentioned variants detailed as in the paper and extended whenever deemed necessary. Category I - part IV omits weighting and only applies an arithmetic mean instead. Category II and Category III are not vital to display the concept of having assailants in a crowd evacuation scenario and only reflect some fine-tuning. Thus, they have been omitted. 

## How to run the simulation

Running the simulation is straightforward. First a simulation instance is created, then a fluent builder API can be used to add the desired additional properties (pedestrians, assailants, obstacles) and at the end you simply call run() on the simulation object to start the simulation. That's it. Various different example scenarios can be found in simulation_tests.py. 

Let's walk through the key options you might want to configure here:
1. Simulation object. You need to provide a FuzzyDomains object. This object contains all membership functions for the fuzzy logic system. The whole system is modular/plug&play and thus any other set of membership functions with its corresponding domain objects (domains are class objects from the chosen underlying fuzzy logic library) can be defined and injected via constructor injection this way. A Goal object with its x and y coordinates has to be provided as well as the second parameter to the simulation constructor. By using the optional scale parameter you may zoom in/out (higher scale value zooms in tighter and vice versa). The optional parameter tick rate defines the speed at which the simulation is run. The tick rate denotes the number of simulation updates which are performed. Thus, a higher tick rate slows down the simulation speed. 
2. add_obstacle() builder method available on the simulation object adds a new obstacle which pedestrians have to avoid. The constructor takes two tuples, i.e. x and y coordinates for each end point. 
3. add_pedestrian() builder method available on the simulation object adds a new pedestrian. The first tuple denotes the x and y coordinates, the next parameter denotes the initial direction the pedestrian is looking at (0 degrees denotes looking to the right, i.e. x-axis oriented to the right denotes 0 degrees in the simulation), the third parameter denotes the initial movement speed of the pedestrian. 
4. add_assailant() build method available on the simulation object adds a predator. The first parameters of its location coordinates, initial direction and speed are defined exactly as for pedestrians but additionally an optional paramater named 'predator_type' may be defined. By default a primitive predator object is defined which means that the predator will simply start to chase and attack the nearest pedestrian based on Euclidean distance. By definining 'predator_type="Advanced"' a more sophisticated version of a predator may be instantiated which will chase and eliminate the most isolated pedestrian based on its Euclidean distance to all other pedestrians in the simulation.
5. run() on the simulation object finally runs the simulation in a pop-up window as configured. 



