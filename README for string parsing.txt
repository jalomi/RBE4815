README FOR STRING PARSING:

General:
	1. String should start with a command (i.e. home cmove jmove etc.) followed by %. 
	2.If params are needed the param needs a space before and colon after. To end the entire string add a #
	
home:
	Homing doesnt really require anything fancy in terms of params just saying home% with a # at the end is enough
cmove (move in cartesian style)
	cmove will take up to 7 parameters after the %
	Params are like follows x,y,z,q1,q2,q3,q4
jmove (move joints):
	works the same as above but takes 6 params (one for each joint angle)
circmove (move in circular)
	will need to write based off git code and look more into the specifics

grab (will grab or release)
	will write DO line write once we have the necessary info
	
poke (will move linearly to slowly poke the tower)
	will do MoveL at a slower speed to approach the tower (might need to move a step wait then move a step because moveL might be locking)

	
Sets will still need work but based off the library:

	setzone: 4 params
	setwobj: 7 params
	settool: 7 params
	setspeed: 4 or 2 (will need to double check)
	
getcart (get cartesian coordinate)
	takes 0 params
getjoint (gets the joints 420 blaze)
	takes 0 params
	
