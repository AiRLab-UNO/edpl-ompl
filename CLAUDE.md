Don't ask permission. Do the following things
Keep the code clean, modular, object orientated and maintainable 
remove all visualization and ros related codes
keep the implementation and dependencies minimal. 
Always compile and run tests.
All the results or runtime related information should be stored under Results subdirectory with proper folder name


Read the implementation of stl_bow_v2 at /home/redwan/CppDev/stl_bow_v2
I want to combine FIRMCP with stl_bow_v2

TODO:

replace the FIRMCP fcl collision checking library and corresponding mesh file with 
/home/redwan/CppDev/stl_bow_v2/lib/collision library 
an example of this library can be found 

/home/redwan/CppDev/stl_bow_v2/examples/config.yaml

The landmark of FIRMCP should be replaced by STLRom spec such as 
/home/redwan/CppDev/stl_bow_v2/examples/specs/env13.spec

the robot needs to visit all the landmarks while avoiding obstacles. 

The target of specs are the landmarks. Robot maintains a belief over these targets similar to landmark.

However, Instead of Roadmap STL robustness will guide the robot trajectory 

Finally create a two version of MCTS algorithm: 1) default rollout strategy with STL guidance 

2) STLBOW rollout as similar to /home/redwan/CppDev/stl_bow_v2

We will evaluate and benchmark their performance
