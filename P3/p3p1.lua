--This file is built around the code that was provided
darwin = require('hfaDriver')

targetY = 0
targetX = 0
targetLimit = 0.05
angleLimit = 10
turnSpeed = 0.1
walkSpeed = 0.05
locTime = 10 -- time to localize, in seconds
radToDeg = 180/math.pi

--functions
function getTurnAngleToTarget(tx, ty) 	
	local angle = 0
	local rx = wcm.get_pose().x
	local ry = wcm.get_pose().y
	local ra = wcm.get_pose().a*radToDeg
	local dx = tx - rx
	local dy = ty - ry
	
	if dx == 0 then --if angle is +/- 90
		angle = 0
	else
		angle = math.atan(dy/dx)*radToDeg
	end
	
	if dx <= 0 and dy <= 0 then -- if in 4th quadrant, adjust tan
		angle = angle - 180
	elseif dx < 0 and dy > 0 then
		angle = angle + 180
	end
	
	angle = angle - ra
	--print ("turning: Rx: "..rx.." Ry: "..ry.." Ra: "..ra.." Ta: "..angle)
	return angle
end

--Behaviors
localizeStart = function(hfa)                   
	print("localize behavior");
	darwin.lookGoal()
	darwin.setVelocity(walkSpeed, walkSpeed,walkSpeed);
end	

localizeGo = function(hfa) -- dont change what we're doing until we go to another state
end
localizeStop = function (hfa)
end

turnToAngleStart = function(hfa)   
	local a = getTurnAngleToTarget(targetX, targetY);
	if a > 0 then
		print("turning left");
		darwin.setVelocity(0, 0,turnSpeed);
	elseif a < 0 then
		darwin.setVelocity(0, 0,-1*turnSpeed);
		print("turning right");
	else
		print("stopping");
		darwin.stop();
	end			
end	

turnToAngleGo = function(hfa) -- dont change what we're doing until we go to another state
print ("turning: Rx: "..wcm.get_pose().x.." Ry: "..wcm.get_pose().y.." Ra: "..wcm.get_pose().a.." Tx: "..targetX.." Ty: "..targetY)
end
turnToAngleStop = function (hfa)
	darwin.lookGoal()
	darwin.stop();	
end


walkForwardStart = function(hfa)                   
	print("walk forward");
	darwin.setVelocity(walkSpeed, 0,0);
end	

walkForwardGo = function(hfa)
	print ("forward: Rx: "..wcm.get_pose().x.." Ry: "..wcm.get_pose().y.." Ra: "..wcm.get_pose().a.." Tx: "..targetX.." Ty: "..targetY)
end
walkForwardStop = function (hfa)
	darwin.lookGoal()
end

stopStart = function(hfa)
	print("i stopped at location " .. wcm.get_pose().x .. ", " .. wcm.get_pose().y .. ", " .. wcm.get_pose().a);
	--print(" the ball is at location " .. wcm.get_ball_x() .. ", " .. wcm.get_ball_y());
	darwin.stop();	
	darwin.track(); -- stares at the last location we saw the ball
end

stopGo = function(hfa)
end
stopStop = function(hfa)
end

-- MAKE STATES
turnToAngle = makeBehavior("turnToAngle", turnToAngleStart, turnToAngleStop, turnToAngleGo);
walkForward = makeBehavior("walkForward", walkForwardStart, walkForwardStop, walkForwardGo);
localize = makeBehavior("localize", localizeStart, localizeStop, localizeGo);
stop = makeBehavior("stop", stopStart, stopStop, stopGo);
oldtime = os.clock()


-- STATE TRANSITIONS
myMachine = makeHFA("myMachine", makeTransition(
	{
		[start] = localize, --first thing we do: walk forward
		[localize] = function()
			if (os.clock() - oldtime)<locTime then
				return localize 
			else 
				return turnToAngle 
			end 
		end,
		[turnToAngle] = function() 
			if math.abs(getTurnAngleToTarget(targetX, targetY))>angleLimit then
				darwin.lookGoal()
				return turnToAngle 
			else 
				return walkForward 
			end 
		end,
		[walkForward] = function() 
			local distance = math.sqrt(math.pow((targetX -wcm.get_pose().x),2)+math.pow((targetY - wcm.get_pose().y),2))
			if math.abs(getTurnAngleToTarget(targetX, targetY)) > angleLimit*3 then
				darwin.lookGoal()
				return turnToAngle
			elseif distance > targetLimit then
				return walkForward 
			else 
				return stop 
			end 
		end,			
		[stop] = function()  
			if math.abs(getTurnAngleToTarget(targetX, targetY))>angleLimit * 2 then 
				return stop 
			else 
				return stop  
			end 
		end,
}),false);	
	
--start "main"
darwin.executeMachine(myMachine);



--Walk to position, face target. Your darwin should be able to walk to a point (x,y) on the field, and face some other point (x’,y’) .
--	while 1
--		wcm.get_pose(): gets x and y angular position. 
--		new speed x, y = update final speed using a bang bang routione so either 0.05 speed or 0
--		new angular = update final angular speed based on our position
--		darwin.setVelocity(new speed x, y, new angular);
--		keep doing this until we get close to the point. 

--Walk towards a ball. Get the darwin close to a ball, and stop.
-- Keep in mind, you may not be able to see the ball initially, you must find the ball then walk toward it.
--	while 1
--		randomly move around the field and the go to ball position?
--		darwin.scan() then wcm.get_ball_x() and wcm.get_ball_y(): 
--		onceball location is found use method 1 to go there.

--Approach Ball and kick. This is not the same as walking towards a ball and kicking! you must align yourself 
--		so that you can kick the ball straight forward with one of your feet.
--		I recommend making your robot either left-footed or right-footed.
--	Use method 2 and get near the ball. Keep updating so that you can get very close. 

--Walk to a ball , approach and kick to target.
--		use method 3 but approach the point in a certain angle so that you can kick it at the end. 

--Walk to a position, wait until ball is close, then approach and kick into the goal.
--		Self explanatory method 1 and 4 combined but use kick to goal look goal?

--Wow me 
