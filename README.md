The Human-Robot Interaction API is a high level, domain specific API designed to make it easy to create human-robot interaction applications. It contains high level primitives for: controlling robot actions, subscribing to social communication and representing and querying perception data.

#1. What does the Human-Robot Interaction API do? Why should I use it?
APIs for programming robots are typically low-level and have little correspondence to how a person would normally describe an interaction between person and robot. The Human-Robot Interaction API provides high level, domain specific primitives that make it easy to program human-robot interaction. It allows you to do things like...

Make a robot:
* speak
* gaze at a persons head
* smile, frown, open mouth...
* point at a perons torso
* wave at someone
* listen to speech
* any combination of any of the above, at the same time or one after the other

Find out:
* when a person is perceived by a robot?
* what did they say?
* who is closest to the robot?
* who is to the left of the robot?
* what entities the robot has perceived?

The Human-Robot Interaction API 

#2. Tutorials
The Human-Robot Interaction API provides a number of important classes you will use when programming human-robot interaction: _Entity_ (with subclasses _Robot_ and _Person_), _World_ and _Query_. The _Robot_ class provides primitives for controlling robot actions and subscribing to social communication. Action primitive’s communicate with vendor specific implementations via a standardised action interface. _Person_ instances model the human body and are used to program interactions with people, e.g. look at a person’s left arm. The _World_ class is a collection of entities perceived by the robot; populated by vendor specific perception algorithms. Lastly, the _Query_ class provides a high level interface, similar to Microsoft’s LINQ to filter entities in the _World_. 


#3. API Documentation


![](https://rawgit.com/uoa-robotics/hri/master/hri/documentation/api_overview.svg)

## Tutorials for supported robots:
The following list contains tutorials that explain how to program human-robot interaction for a number of different robots, including: 

* [Nao](https://github.com/uoa-robotics/nao_hri/wiki)
* [Zeno]()
* [Zoidstein]()
* [Robokind R50]()

## Making re-usable behavioural modules:

## Adding a new robot:
The following links detail the steps needed to add support for a new robot or algorithm to hri api.
