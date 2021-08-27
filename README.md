# Gantry-Path-Planning

Path planning code for the MATRIX project. Utilizes D* Lite based on Koenig, S. and Likhachev, M.'s paper **__D* Lite__**, 2002 to path plan through the space.

[Link to paper](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf)


This project operates in a completely known environment. First, a large grid is generated, and then obstacles are put in. When obstacles are put in, a finer mesh of coordinates is generated around it to increase precision in path planning. Then, a graph is generated from the grid and obstacles ignoring the obstacles and then D* Lite is used to navigate around the uneven grid.
