package main

import (
	"math"

	cy "clunkyard"
)

const (
	searchArc = 40.0 // degrees: sweeps the circle in half a second
	holdWidth = 60.0 // units across at the target: it stays in the beam from tick to tick
	minArc    = 6.0  // degrees: the narrowest the beam holds
	window    = 12   // ticks of sightings the aim averages
	sightings = 8    // narrow sightings in the window before a shot
	lookEvery = 40   // ticks between looks around the target, for a nearer robot
	reacquire = 10   // ticks to look again where a lost target was
)

type starter struct {
	freeFlight         bool
	dodgeTicks, dodges int
	slot               int32
}

func must[T any](out T, err error) T {
	if err != nil {
		panic(err)
	}
	return out
}

func check(err error) {
	if err != nil {
		panic(err)
	}
}

func steer(r *cy.Robot, towards, current float64) float64 {
	return must(r.Angles.Clamp(must(r.Angles.AngleDiff(towards, current))/45, -1, 1))
}

func drive(r *cy.Robot, throttle, turn, pitch float64) {
	check(r.Drive.Set(cy.DriveSet{Throttle: cy.Num(throttle), Turn: cy.Num(turn), PitchTurn: cy.Num(pitch)}))
}

func scan(r *cy.Robot, heading, arc, pitch float64) {
	check(r.Scanner.Set(cy.ScannerSet{Heading: cy.Num(heading), Arc: cy.Num(arc), Pitch: cy.Num(pitch)}))
}

// fit is a straight line through the narrowest of the last window ticks'
// sightings: where the robot is now and how fast it moves. Each sighting is off
// by its BearingError; the line averages that out.
func fit(r *cy.Robot, slot, tick int32) (position, velocity cy.Vec3, count int) {
	best := math.Inf(1)
	recent := []cy.Sighting{}
	for _, s := range must(r.Tracker.History(slot)) {
		if s.Tick > tick-window {
			recent = append(recent, s)
			best = math.Min(best, s.BearingError)
		}
	}
	if len(recent) < 2 {
		return cy.Vec3{}, cy.Vec3{}, 0
	}
	seen := []cy.Sighting{}
	for _, s := range recent {
		if s.BearingError <= 1.5*best {
			seen = append(seen, s)
		}
	}
	if len(seen) < 2 {
		return cy.Vec3{}, cy.Vec3{}, 0
	}
	n := float64(len(seen))
	t0, spread := 0.0, 0.0
	for _, s := range seen {
		t0 += float64(s.Tick) / n
	}
	for _, s := range seen {
		spread += (float64(s.Tick) - t0) * (float64(s.Tick) - t0)
	}
	rate := float64(must(r.Match.TickRate()))
	// Least squares on one axis: its fitted position now and its speed per second.
	line := func(axis func(cy.Vec3) float64) (now, speed float64) {
		mean, along := 0.0, 0.0
		for _, s := range seen {
			mean += axis(s.Position) / n
		}
		for _, s := range seen {
			along += (float64(s.Tick) - t0) * (axis(s.Position) - mean)
		}
		slope := along / spread
		return mean + slope*(float64(tick)-t0), slope * rate
	}
	position.X, velocity.X = line(func(v cy.Vec3) float64 { return v.X })
	position.Y, velocity.Y = line(func(v cy.Vec3) float64 { return v.Y })
	position.Z, velocity.Z = line(func(v cy.Vec3) float64 { return v.Z })
	return position, velocity, len(seen)
}

func (s *starter) Start(r *cy.Robot) error {
	s.freeFlight = must(r.Match.Is3d()) && must(r.Arena.Gravity()).Z == 0
	return nil
}

func (s *starter) Turn(r *cy.Robot) error {
	position, heading, pitch, tick := must(r.Me.Position()), must(r.Me.Heading()), must(r.Me.Pitch()), must(r.Clock.Tick())
	check(r.Drive.Forces(cy.DriveForces{Brake: cy.Num(0.2)}))
	var lastSeen *int32
	if s.slot != 0 {
		lastSeen = must(r.Tracker.LastSeen(s.slot))
	}
	if target := must(r.Sensors.Nearest()); target != nil {
		s.slot = target.Slot
		throttle := 0.2
		if target.Range > 350 {
			throttle = 0.7
		} else if target.Range < 200 {
			throttle = -0.4
		}
		drive(r, throttle, steer(r, target.Bearing, heading), steer(r, target.Elevation, pitch))
		// Halve the beam each tick until it is holdWidth across at the target:
		// narrow enough for a precise bearing, wide enough to keep it in.
		hold := math.Max(minArc, 2*math.Atan2(holdWidth/2, target.Range)*180/math.Pi)
		// In a crowd a narrow beam sees one robot. Every lookEvery ticks, widen it
		// for a tick: a nearer robot it finds becomes the target.
		arc := math.Max(hold, math.Min(searchArc, must(r.Scanner.Last()).Arc/2))
		if tick%lookEvery == 0 {
			arc = searchArc
		}
		scan(r, target.Bearing, arc, target.Elevation)
		power := must(r.Gun.PowerForFlightTime(target.Range, 1))
		// An exact contact: Gun.Lead is the intercept if the target holds course.
		// A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
		var aim *cy.Solution
		ready := true
		if target.BearingError > 0 {
			position, velocity, count := fit(r, target.Slot, tick)
			if count > 0 {
				aim = must(r.Gun.LeadPoint(position, velocity, power))
			}
			ready = aim != nil && count >= sightings
		} else {
			aim = must(r.Gun.Lead(*target, power))
		}
		towards, up := target.Bearing, target.Elevation
		if aim != nil {
			towards, up = aim.Heading, aim.Pitch
		}
		check(r.Gun.Aim(cy.GunAim{Heading: cy.Num(towards), Pitch: cy.Num(up)}))
		now := must(r.Gun.AimNow())
		if ready && math.Abs(must(r.Angles.AngleDiff(towards, now.Heading))) < 3 && math.Abs(up-now.Pitch) < 3 && must(r.Gun.CanFire(power, 10)) {
			check(r.Gun.Fire(cy.GunFire{Power: cy.Num(power)}))
		}
		text := "NARROW"
		if ready {
			text = "TRACK"
		}
		check(r.Status.Set(cy.StatusSet{Text: cy.Text(text), Value: cy.Num(target.Range)}))
	} else if lastSeen != nil && tick-*lastSeen <= reacquire {
		// Lost it: look again where it was last seen, widening the beam, before searching.
		where := must(r.Tracker.Position(s.slot))
		bearing := must(r.Angles.BearingTo(position, *where))
		elevation := 0.0
		if s.freeFlight {
			elevation = must(r.Angles.ElevationTo(position, *where))
		}
		drive(r, 0.5, steer(r, bearing, heading), steer(r, elevation, pitch))
		scan(r, bearing, math.Min(searchArc, must(r.Scanner.Last()).Arc*2), elevation)
		check(r.Status.Set(cy.StatusSet{Text: cy.Text("REACQUIRE")}))
	} else if cue := must(r.Tracker.Cue()); cue != nil {
		bearing, distance := must(r.Angles.BearingTo(position, cue.Position)), must(r.Angles.Distance(position, cue.Position))
		elevation := 0.0
		if s.freeFlight {
			elevation = must(r.Angles.ElevationTo(position, cue.Position))
		}
		error := math.Min(cue.BearingError, 30)
		across := float64(tick%5-2) / 2 * error
		up := 0.0
		if s.freeFlight {
			up = float64((tick/5)%3-1) * error / 2
		}
		drive(r, 0.7, steer(r, bearing, heading), steer(r, elevation, pitch))
		scan(r, bearing+across, must(r.Scanner.ArcForReach(distance+100)), elevation+up)
		check(r.Status.Set(cy.StatusSet{Text: cy.Text("CUE"), Value: cy.Num(distance)}))
	} else {
		drive(r, 0.5, 0.33, steer(r, 0, pitch))
		up := 0.0
		if s.freeFlight {
			up = float64((tick/10)%3-1) * 35
		}
		scan(r, must(r.Angles.Normalize(must(r.Scanner.Last()).Heading+36)), searchArc, up)
		check(r.Status.Set(cy.StatusSet{Text: cy.Text("SEARCH")}))
	}

	fired := false
	for _, heard := range must(r.Sensors.Cues()) {
		fired = fired || heard.Source == cy.SourceMuzzle
	}
	if s.dodgeTicks == 0 && fired {
		s.dodges++
		s.dodgeTicks = 10
	}
	if s.dodgeTicks > 0 {
		// Odd dodges go left, even ones right; turn +1 and strafe -1 go left.
		left := s.dodges%2 == 1
		side, text := -1.0, "dodge:right"
		if left {
			side, text = 1, "dodge:left"
		}
		s.dodgeTicks--
		if must(r.Match.Control()) == cy.ControlForces {
			check(r.Drive.Set(cy.DriveSet{Throttle: cy.Num(0.5)}))
			check(r.Drive.Forces(cy.DriveForces{Strafe: cy.Num(-side)}))
		} else {
			check(r.Drive.Set(cy.DriveSet{Throttle: cy.Num(1), Turn: cy.Num(side)}))
		}
		check(r.Status.Set(cy.StatusSet{Text: cy.Text(text), Value: cy.Num(float64(s.dodges))}))
	}
	if must(r.Battery.Charge()) < must(r.Battery.Capacity())/4 {
		if throttle := must(r.Drive.GetThrottle()); throttle != nil {
			check(r.Drive.Set(cy.DriveSet{Throttle: cy.Num(*throttle * 0.5)}))
		}
	}
	if storm := must(r.Nav.Storm()); storm != nil {
		if margin := must(storm.Margin(position)); margin != nil && *margin > -60 {
			away := must(r.Angles.BearingTo(storm.Center, position))
			check(r.Drive.Set(cy.DriveSet{Throttle: cy.Num(0.8), Turn: cy.Num(must(r.Angles.Clamp(must(r.Angles.AngleDiff(away, heading))/30, -1, 1)))}))
		}
	}
	if zone := must(r.Nav.Zone()); zone != nil {
		if margin := must(zone.Margin(position)); margin != nil && *margin < 60 {
			bearing := must(r.Angles.BearingTo(position, zone.Center))
			climb := 0.0
			if zone.Shape == cy.ShapeSphere {
				climb = must(r.Angles.ElevationTo(position, zone.Center))
			}
			drive(r, 0.8, must(r.Angles.Clamp(must(r.Angles.AngleDiff(bearing, heading))/30, -1, 1)), must(r.Angles.Clamp((climb-pitch)/30, -1, 1)))
		}
	}
	return nil
}

func main() {
	if err := cy.Run(&starter{}); err != nil {
		panic(err)
	}
}
