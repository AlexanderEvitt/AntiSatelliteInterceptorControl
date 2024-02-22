function findInterceptPoint {
	// Returns the time of the intercept point
  set a to TIME.
  set b to TIME + TIMESPAN(0,0,0.2).
	set phi to (sqrt(5) - 1) / 2.

  until b-a < 0.01 {
		set d to phi*(b-a).
		set x1 to POSITIONAT(TARGET,a+d):SQRMAGNITUDE.
		set x2 to POSITIONAT(TARGET,b-d):SQRMAGNITUDE.

		if x1 < x2 {
			set a to b-d.
		}
		if x1 > x2 {
			set b to a+d.
		}
  }
  return a.
}

function findPitchRate {
	parameter t_i.
	set crossAngle to VECTORANGLE(UP:vector,POSITIONAT(TARGET,t_i)).
	return crossAngle*0.025. // I love magic numbers
}

function findThrustDir {
	// This function calculates the thrust direction at a given instant in time after Launch
	// To do: make this turn in the correct direction
	parameter t_i.
	parameter t.
	set pitch to findPitchRate(t_i)*t.
	set about to VELOCITYAT(target,t_i):surface:normalized.
	return (ANGLEAXIS(-pitch,about)*UP).
}

function findTimeOfFlight {
	// This function calculates the time of flight.
	parameter t_i.
	set interceptPosition to POSITIONAT(TARGET,t_i).
	set n1 to 600.
	set n2 to 200.
	set dt1 to 59.1/n1.
	set dt2 to 8.8/n2.
	set dt3 to 0.05.
	set currentPosition to V(0,0,0).
	set currentVelocity to V(0,0,0).
	set positionList to list(currentPosition).
	set velocityList to list(currentVelocity).
	set currentMass to 45818.
	set m_0 to currentMass.
	set m_1 to 11018.

	// First, numerically integrate for the first stage:
	for i in range(0,n1,1) {
		set acceleration to (1200000/currentMass)*findThrustDir(t_i,i*dt1):vector - (1.2)*UP:vector - 2.5105*(10^13)*(currentPosition - ship:body:position):normalized/(currentPosition - ship:body:position):sqrmagnitude.
		set currentVelocity to currentVelocity + acceleration*dt1.
		set currentPosition to currentPosition + currentVelocity*dt1.
		set currentMass to currentMass + (m_1 - m_0)/n1.

		positionList:add(currentPosition).
		velocityList:add(currentVelocity).
	}

	// Second, numerically integrate for the second stage:
	set currentMass to 2085.
	set m_1 to currentMass.
	set m_2 to 1035.
	for i in range(0,n2,1) {
		set acceleration to (192000/currentMass)*findThrustDir(t_i,n1*dt1 + i*dt2):vector - (1)*UP:vector - 2.5105*(10^13)*(currentPosition - ship:body:position):normalized/(currentPosition - ship:body:position):sqrmagnitude.
		set currentVelocity to currentVelocity + acceleration*dt2.
		set currentPosition to currentPosition + currentVelocity*dt2.
		set currentMass to currentMass + (m_2 - m_1)/n2.

		positionList:add(currentPosition).
		velocityList:add(currentVelocity).
	}

	// Finally, numerically integrate for the thrustless flight until the interceptor is farther from the launch site than the intercept point
	set currentTime to n1*dt1 + n2*dt2.
	until currentPosition:sqrmagnitude > interceptPosition:sqrmagnitude {
		set acceleration to -2.5105*(10^13)*(currentPosition - ship:body:position):normalized/(currentPosition - ship:body:position):sqrmagnitude.
		set currentVelocity to currentVelocity + acceleration*dt3.
		set currentPosition to currentPosition + currentVelocity*dt3.

		set currentTime to currentTime + dt3.
	}
	print currentVelocity.
	return currentTime.
}

function homing {
	rcs off.
	set referenceVec to lookdirup(V(0,0,1),V(-1,0,0)).
	set pointing to ROTATEFROMTO(referenceVec:VECTOR,TARGET:POSITION).
	lock steering to pointing*referenceVec.
	print "Steering.".
	TOGGLE AG1.

	// Returns the time of the closest approach point
  set a to TIME.
  set b to TIME + TIMESPAN(0,0,0,5).
	set phi to (sqrt(5) - 1) / 2.

  until b-a < 0.01 {
		set d to phi*(b-a).
		set x1 to (POSITIONAT(TARGET,a+d) - POSITIONAT(SHIP,a+d)):SQRMAGNITUDE.
		set x2 to (POSITIONAT(TARGET,b-d) - POSITIONAT(SHIP,b-d)):SQRMAGNITUDE.

		if x1 < x2 {
			set a to b-d.
		}
		if x1 > x2 {
			set b to a+d.
		}
  }
	print "The intercept time is at " + (a - time):seconds.

	wait 2.
	set t_i to a.
	rcs on.

	SET xPID TO PIDLOOP(0.1, 0, 0).
	SET xPID:SETPOINT TO 0.
	SET yPID TO PIDLOOP(0.1, 0, 0).
	SET yPID:SETPOINT TO 0.
	SET zPID TO PIDLOOP(0.1, 0, 0).
	SET zPID:SETPOINT TO 0.


	until time > t_i {
		set pointing to ROTATEFROMTO(referenceVec:VECTOR,TARGET:POSITION).
		lock steering to pointing*referenceVec.
		set x_i to POSITIONAT(TARGET,t_i) - ship:body:position.
		set currentPosition to POSITIONAT(ship,t_i) - ship:body:position.
		set error to currentPosition - x_i.

		set controlInput to V(xPID:UPDATE(time:seconds,error:x), yPID:UPDATE(time:seconds,error:y), zPID:UPDATE(time:seconds,error:z)).
		set firings to pointing:inverse*controlInput.
		set ship:control:fore to firings:z.
		set ship:control:starboard to firings:y.
		set ship:control:top to -1*firings:x.

		readout(t_i, time, "Terminal Homing", error).

		set limit to 5*ship:monopropellant.
		set engList to ship:partstagged("homing").
		for eng in engList { set eng:thrustlimit to limit. }
	}

	set ship:control:fore to 0.
	set ship:control:starboard to 0.
	set ship:control:top to 0.
}

function readout {
	parameter t_i.
	parameter t_l.
	parameter phase.
	parameter error.
	LOCAL termWidth IS TERMINAL:WIDTH.
	LOCAL termHeight IS TERMINAL:HEIGHT.
	if time > t_l {
		set pitchRate to findPitchRate(t_i).
	}
	else {
		set pitchRate to 0.
	}

	clearscreen.
	PRINT ("Current Phase: " + phase):PADRIGHT(termWidth) AT(0,0).print("").
	PRINT ("Time to Intercept: " + floor((t_i - time):seconds) + "s"):PADRIGHT(termWidth) AT(0,1).print("").
	if time > t_l {
		PRINT ("Time From Launch: " + floor((time - t_l):seconds) + "s"):PADRIGHT(termWidth) AT(0,2).print("").
	}
	else {
		PRINT ("Time to Launch: " + floor((t_l - time):seconds) + "s"):PADRIGHT(termWidth) AT(0,2).print("").
	}
	PRINT ("__________________________________________________") AT(0,3).

	PRINT ("Pitch Rate: " + round(pitchRate,3) + "deg/s"):PADRIGHT(termWidth) AT(0,4).print("").
	PRINT ("Commanded Steering Angle: " + floor(pitchRate*(time-t_l):seconds) + "deg"):PADRIGHT(termWidth) AT(0,5).print("").
	PRINT ("__________________________________________________") AT(0,6).

	PRINT ("Target Object: " + target:shipname):PADRIGHT(termWidth) AT(0,7).print("").
	PRINT ("Target Ground Speed: " + floor(target:groundspeed) + "m/s"):PADRIGHT(termWidth) AT(0,8).print("").
	PRINT ("Target Altitude: " + floor(target:altitude) + "m"):PADRIGHT(termWidth) AT(0,9).print("").
	PRINT ("__________________________________________________") AT(0,10).

	if error:mag > 0 {
		PRINT ("O") AT(termWidth/2,(termHeight/2)+3).
		set xloc to (error:x/abs(error:x))*(abs(error:x)^(1/4)).
		set yloc to (error:y/abs(error:y))*(abs(error:y)^(1/4)).
		if ROUND(xloc) = 0 and ROUND(yloc) = 0 {
			PRINT("(X)") at(termWidth/2,(termHeight/2)+3).
		} else {
			PRINT "X" AT((termWidth/2)+FLOOR(xloc),(termHeight/2)+FLOOR(yloc)+3).
		}
	}

	wait 0.
}


function main {
	wait until hastarget = true.
	set launch to false.
	until launch = true {
		set t_i to findInterceptPoint.
		if t_i < time + 10 {
			clearscreen.
			print "No near intercept.".
		}
		else if t_i - time > 700 {
			clearscreen.
			print "No near intercept.".
		}
		else {
			set t_f to findTimeOfFlight(t_i).

			set t_l to (t_i - t_f).

			readout(t_i, t_l, "Pre-Launch", V(0,0,0)).
			if time+10 > t_l {
				set launch to true.
			}
		}
	}

	until time > t_i - t_f {
		readout(t_i,t_l,"Terminal Count", V(0,0,0)).
	}
	SET STEERINGMANAGER:PITCHPID:KP TO 1.15.
	SET STEERINGMANAGER:PITCHPID:KI TO 0.7.
	SET STEERINGMANAGER:PITCHPID:KD TO 0.1.
	SET STEERINGMANAGER:MAXSTOPPINGTIME TO 1.

	rcs on.
	lock steering to findThrustDir(t_i,(time-t_l):seconds).
	stage.
	until time > t_l + 59.1 + 0.2 {
		readout(t_i, t_l, "First Stage Burn", V(0,0,0)).
	}
	stage.
	until time > t_l + 59.1 + 8.8 + 0.1 {
		readout(t_i, t_l, "Second Stage Burn", V(0,0,0)).
	}
	stage.

	homing().
}


main().
