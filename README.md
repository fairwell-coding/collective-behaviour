# Collective behaviour - group A

## Public GIT repository 

The public GIT repository can be found under: https://github.com/fairwell-coding/collective-behaviour

## Chosen paper

The project is based on the article: Modeling of Crowd Evacuation With Assailants via a Fuzzy Logic Approach (Zhou, Dong, Wen, Yau and Sun, 2015)

We will use the outlined fuzzy-logic approach in the paper as our starting point. 

## Rough sketch w.r.t 3 report deadlines

Our plan is to model a system, where a group of *n* pedestrians tries to escape a group of *m* assailants in a closed environment with randomly placed obstacles (a building or a maze). Our final goal would be to find an optimal survival strategy for the pedestrians. 

Initially, we would limit ourselves to only 1 assailant, modeling the environment ourselves and using an instruction set modeled on fuzzy logic to provide instructions for the pedestrians. Then, we would improve our model by increasing the number of assailants, building our model for a general closed environment, and implementing our pedestrians' survivability instinct using machine learning.

Specifically, for phase II and III we intend to explore suitable ML approaches based on genetic algorithms and compare the outcomes with the original fuzzy logic approach. Implementation will be based on either ML agents or our own project setup loading the final trained Pytorch models ourselves. We will also try to find suitable deep reinforcement approaches w.r.t. computational requirements. 


