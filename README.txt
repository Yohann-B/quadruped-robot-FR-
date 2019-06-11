*------------------------------------------------------------------------------*
                            Quadrupede robot project
                                  README file
*------------------------------------------------------------------------------*
The file in this depositery are in french.

The goal is to use some 3D blueprints found on the internet to create a 
quadrupede robot using low level c++ programming.

The project is using the ATmega2560 that we can find on the arduino mega board.

The part with the HC-SR03 distance's sensor is not yet implemented. Before, a
redesign of the robot is necessary since the prototype with the huge board is 
not providing good results.

*------------------------------------------------------------------------------*
#File description:

	"RQ_rapport.pdf"
This file is the report that explain the creation of the robot and the different
choices that were made.

	"main.cpp"
This file contain all the functions and the main code of the project. 
To perform different sequence of event, the user should put the different 
functions created in a certain order (or with additionnal rules) in the "while"
loop.
*------------------------------------------------------------------------------