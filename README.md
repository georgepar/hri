The Human-Robot Interaction API is a high level, domain specific API designed to make it easy to create human-robot interaction applications. It contains high level interfaces for: controlling robot actions, subscribing to social communication and representing and querying perception data. A high level overview of the API is illustrated below.

![](https://rawgit.com/uoa-robotics/hri/master/hri/documentation/api_overview.svg)

The API provides a number of important classes that perform different tasks: _Entity_ (with subclasses _Robot_ and _Person_), _World_ and _Query_. The _Robot_ class provides primitives for controlling robot actions and subscribing to social communication. The action primitive’s communicate with vendor specific implementations via a standardised action interface. _Person_ instances model the human body and are used to program interactions with people, e.g. look at a person’s left arm. The _World_ class is a collection of entities perceived by the robot; populated by vendor specific perception algorithms. Lastly, the _Query_ class provides a high level interface, similar to Microsoft’s LINQ to filter entities in the _World_. 

## Tutorials for supported robots:
The following list contains tutorials that explain how to program human-robot interaction for a number of different robots, including: 

* [Nao](https://github.com/uoa-robotics/nao_hri/wiki)
* [Zeno]()
* [Zoidstein]()
* [Robokind R50]()

## Making re-usable behavioural modules:

## Adding a new robot:
The following links detail the steps needed to add support for a new robot or algorithm to hri api.

*  
*
*
