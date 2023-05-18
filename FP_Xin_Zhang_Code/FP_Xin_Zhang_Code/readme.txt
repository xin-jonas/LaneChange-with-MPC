A. File Description

1.'Highway_maneuever_simulation.m' is main file, run this file to start simulation
In this File, the initial velocity and position of two vehicles can be changed.
2.'Vehilces.m': generate the trajectory of target vehicle with constant speed
'nmpc.m': mpc controller 
3.'terminalcost1v1.m': calculate terminal cost
'runningcost.m' or 'runningcost1v1.m': calculate runnigcost based the reference, 
which is calculated from 'ManueverGeneration_1.m'
4.'ManueverGeneration.m': select one manuever and generate the reference (goal lane and goal velocity)
, which will be used to calculate cost
5.'TTC_TIV_1.m': use TTC and TIV to determine if the ego vehicle can change lane. 
The output of the function 'LC' will be used to set goal lane 
6.'VelocitySet.m':this function will be called, if ego vehicle can change lane. 
And the goal velocity can be setting 
7.'plotVariable.m': plot the figure for trajectory, velocity and acceration after simulation to help analyse

B. Program Struct for main part:
- Highway_maneuever_simulation.m
    -Vehilces.m
    -nmpc.m
        -terminalcosts1v1.m
        -runnningcosts1v1.m
            -ManueverGeneration_1.m
                -TTC_TIV_1.m
                -VelocitySet.m

C. Different version for different manurver update frequence 
version_1 
Maneuver update at each prediction step at each iteration

version_2
Maneuver update at each iteration

version_3
set the paramter in file 'runningcosts1v21.m' to change frequence

mod(a,b): change b to set frequence
b = 10: Maneuver update at every 10 prediction steps at each iteration 
b = 5:Maneuver update at every 5 prediction steps at each iteration
b = 2:Maneuver update at every 2 prediction steps at each iteration

version_4
set the paramter in file 'runningcosts.m' to change frequence

mod(a,b): change b to set frequence
b = 10: Maneuver update at every 10 iterations
b = 5:Maneuver update at every 5 iterations
b = 2:Maneuver update at every 2 iterations
