1. instruction
This pri contains all perception algorithms.

2.
PerceprionInterface is the base class for perception. This class contains ultility variables and function.
There is a class to perform perception and a class name with "base" for every algorithm. The base class is the bridge between original
cloud and specifical algorithm.

3. polymorphism
I create PerceptionInterface pointer that point to an object of a base class. The object will be specified when program starts and is
set in file "/config/config.ini".
