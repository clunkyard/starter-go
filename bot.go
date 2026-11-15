// A robot on the Clunkyard core's Go binding. The core owns tracking,
// geometry and protocol handling; this strategy chooses the tactics.
// It searches with a broad scan, narrows the beam on a contact, and leads
// shots from a line through its recent sightings.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied hull_mounts to_mount(heading pitch)
//   from_mount(heading pitch) mount_toward(point)
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   history(slot) last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) to_chassis(heading pitch attitude)
//   from_chassis(heading pitch attitude) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
package main

import (
    cy "clunkyard"
    "math"
)
// Bearing is earned: a scan contact's bearing is off by up to its BearingError
// (degrees, a fixed share of the arc), so a wide beam finds a robot and a narrow
// one aims at it. Contacts within 120 units are exact.
const (
    searchArc=40.0  // degrees: sweeps the circle in half a second
    holdWidth=60.0  // units across at the target: it stays in the beam from tick to tick
    minArc=6.0      // degrees: the narrowest the beam holds
    window=12       // ticks of sightings the aim averages
    sightings=8     // narrow sightings in the window before a shot
    lookEvery=40    // ticks between looks around the target, for a nearer robot
    reacquire=10    // ticks to look again where a lost target was
)
type starter struct { freeFlight bool; dodgeTicks,dodges int; slot int32 }
func value[T any](out T, err error) T { if err != nil { panic(err) }; return out }
func check(err error) { if err != nil { panic(err) } }
func steer(r *cy.Robot, towards, current float64) float64 { return value(r.Angles.Clamp(value(r.Angles.AngleDiff(towards,current))/45,-1,1)) }
func drive(r *cy.Robot,throttle,turn,pitch float64) { check(r.Drive.Set(cy.DriveSet{Throttle:cy.Num(throttle),Turn:cy.Num(turn),PitchTurn:cy.Num(pitch)})) }
func scan(r *cy.Robot,heading,arc,pitch float64) { check(r.Scanner.Set(cy.ScannerSet{Heading:cy.Num(heading),Arc:cy.Num(arc),Pitch:cy.Num(pitch)})) }
// fit is a straight line through the narrowest of the last window ticks'
// sightings: where the robot is now and how fast it moves. Each sighting is off
// by its BearingError; the line averages that out.
func fit(r *cy.Robot,slot,tick int32) (position,velocity cy.Vec3,count int) {
    best:=math.Inf(1);recent:=[]cy.Sighting{}
    for _,s:=range value(r.Tracker.History(slot)) {if s.Tick>tick-window {recent=append(recent,s);best=math.Min(best,s.BearingError)}}
    if len(recent)<2 {return}
    seen:=[]cy.Sighting{};for _,s:=range recent {if s.BearingError<=1.5*best {seen=append(seen,s)}}
    if len(seen)<2 {return}
    n:=float64(len(seen));t0,spread:=0.0,0.0
    for _,s:=range seen {t0+=float64(s.Tick)/n}
    for _,s:=range seen {spread+=(float64(s.Tick)-t0)*(float64(s.Tick)-t0)}
    rate:=float64(value(r.Match.TickRate()))
    // Least squares on one axis: its fitted position now and its speed per second.
    line:=func(axis func(cy.Vec3) float64) (now,speed float64) {
        mean,along:=0.0,0.0
        for _,s:=range seen {mean+=axis(s.Position)/n}
        for _,s:=range seen {along+=(float64(s.Tick)-t0)*(axis(s.Position)-mean)}
        slope:=along/spread;return mean+slope*(float64(tick)-t0),slope*rate
    }
    position.X,velocity.X=line(func(v cy.Vec3) float64 {return v.X})
    position.Y,velocity.Y=line(func(v cy.Vec3) float64 {return v.Y})
    position.Z,velocity.Z=line(func(v cy.Vec3) float64 {return v.Z})
    return position,velocity,len(seen)
}
func (s *starter) Start(r *cy.Robot) error { s.freeFlight=value(r.Match.Is3d())&&value(r.Arena.Gravity()).Z==0; return nil }
func (s *starter) Turn(r *cy.Robot) error {
    position,heading,pitch,tick:=value(r.Me.Position()),value(r.Me.Heading()),value(r.Me.Pitch()),value(r.Clock.Tick())
    check(r.Drive.Forces(cy.DriveForces{Brake:cy.Num(0.2)}))
    var lastSeen *int32;if s.slot!=0 {lastSeen=value(r.Tracker.LastSeen(s.slot))}
    if target:=value(r.Sensors.Nearest());target!=nil {
        s.slot=target.Slot
        throttle:=0.2;if target.Range>350 {throttle=0.7} else if target.Range<200 {throttle=-0.4}
        drive(r,throttle,steer(r,target.Bearing,heading),steer(r,target.Elevation,pitch))
        // Halve the beam each tick until it is holdWidth across at the target:
        // narrow enough for a precise bearing, wide enough to keep it in.
        hold:=math.Max(minArc,2*math.Atan2(holdWidth/2,target.Range)*180/math.Pi)
        // In a crowd a narrow beam sees one robot. Every lookEvery ticks, widen it
        // for a tick: a nearer robot it finds becomes the target.
        arc:=math.Max(hold,math.Min(searchArc,value(r.Scanner.Last()).Arc/2));if tick%lookEvery==0 {arc=searchArc}
        scan(r,target.Bearing,arc,target.Elevation)
        power:=value(r.Gun.PowerForFlightTime(target.Range,1))
        // An exact contact: Gun.Lead is the intercept if the target holds course.
        // A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
        var aim *cy.Solution;ready:=true
        if target.BearingError>0 {
            position,velocity,count:=fit(r,target.Slot,tick)
            if count>0 {aim=value(r.Gun.LeadPoint(position,velocity,power))}
            ready=aim!=nil&&count>=sightings
        } else {aim=value(r.Gun.Lead(*target,power))}
        towards,up:=target.Bearing,target.Elevation;if aim!=nil {towards,up=aim.Heading,aim.Pitch}
        check(r.Gun.Aim(cy.GunAim{Heading:cy.Num(towards),Pitch:cy.Num(up)}));now:=value(r.Gun.AimNow())
        if ready&&math.Abs(value(r.Angles.AngleDiff(towards,now.Heading)))<3&&math.Abs(up-now.Pitch)<3&&value(r.Gun.CanFire(power,10)) {check(r.Gun.Fire(cy.GunFire{Power:cy.Num(power)}))}
        text:="NARROW";if ready {text="TRACK"}
        check(r.Status.Set(cy.StatusSet{Text:cy.Text(text),Value:cy.Num(target.Range)}))
    } else if lastSeen!=nil&&tick-*lastSeen<=reacquire {
        // Lost it: look again where it was last seen, widening the beam, before searching.
        where:=value(r.Tracker.Position(s.slot));bearing:=value(r.Angles.BearingTo(position,*where));elevation:=0.0
        if s.freeFlight {elevation=value(r.Angles.ElevationTo(position,*where))}
        drive(r,0.5,steer(r,bearing,heading),steer(r,elevation,pitch));scan(r,bearing,math.Min(searchArc,value(r.Scanner.Last()).Arc*2),elevation)
        check(r.Status.Set(cy.StatusSet{Text:cy.Text("REACQUIRE")}))
    } else if cue:=value(r.Tracker.Cue());cue!=nil {
        bearing,distance:=value(r.Angles.BearingTo(position,cue.Position)),value(r.Angles.Distance(position,cue.Position))
        elevation:=0.0;if s.freeFlight {elevation=value(r.Angles.ElevationTo(position,cue.Position))}
        error:=math.Min(cue.BearingError,30);across:=float64(tick%5-2)/2*error;up:=0.0;if s.freeFlight {up=float64((tick/5)%3-1)*error/2}
        drive(r,0.7,steer(r,bearing,heading),steer(r,elevation,pitch));scan(r,bearing+across,value(r.Scanner.ArcForReach(distance+100)),elevation+up)
        check(r.Status.Set(cy.StatusSet{Text:cy.Text("CUE"),Value:cy.Num(distance)}))
    } else {
        drive(r,0.5,0.33,steer(r,0,pitch));up:=0.0;if s.freeFlight {up=float64((tick/10)%3-1)*35}
        scan(r,value(r.Angles.Normalize(value(r.Scanner.Last()).Heading+36)),searchArc,up);check(r.Status.Set(cy.StatusSet{Text:cy.Text("SEARCH")}))
    }
    // A simple dodge: vary it, or predict the shooter.
    fired:=false;for _,heard:=range value(r.Sensors.Cues()) {fired=fired||heard.Source==cy.SourceMuzzle}
    if s.dodgeTicks==0&&fired {s.dodges++;s.dodgeTicks=10}
    if s.dodgeTicks>0 {
        // Odd dodges go left, even ones right; turn +1 and strafe -1 go left.
        left:=s.dodges%2==1;side,text:=-1.0,"dodge:right";if left {side,text=1,"dodge:left"};s.dodgeTicks--
        if value(r.Match.Control())==cy.ControlForces {
            check(r.Drive.Set(cy.DriveSet{Throttle:cy.Num(0.5)}));check(r.Drive.Forces(cy.DriveForces{Strafe:cy.Num(-side)}))
        } else {check(r.Drive.Set(cy.DriveSet{Throttle:cy.Num(1),Turn:cy.Num(side)}))}
        check(r.Status.Set(cy.StatusSet{Text:cy.Text(text),Value:cy.Num(float64(s.dodges))}))
    }
    if value(r.Battery.Charge())<value(r.Battery.Capacity())/4 {if throttle:=value(r.Drive.GetThrottle());throttle!=nil {check(r.Drive.Set(cy.DriveSet{Throttle:cy.Num(*throttle*0.5)}))}}
    if storm:=value(r.Nav.Storm());storm!=nil {if margin:=value(storm.Margin(position));margin!=nil&&*margin>-60 {
        away:=value(r.Angles.BearingTo(storm.Center,position));check(r.Drive.Set(cy.DriveSet{Throttle:cy.Num(0.8),Turn:cy.Num(value(r.Angles.Clamp(value(r.Angles.AngleDiff(away,heading))/30,-1,1)))}))
    }}
    if zone:=value(r.Nav.Zone());zone!=nil {if margin:=value(zone.Margin(position));margin!=nil&&*margin<60 {
        bearing:=value(r.Angles.BearingTo(position,zone.Center));climb:=0.0;if zone.Shape==cy.ShapeSphere {climb=value(r.Angles.ElevationTo(position,zone.Center))}
        drive(r,0.8,value(r.Angles.Clamp(value(r.Angles.AngleDiff(bearing,heading))/30,-1,1)),value(r.Angles.Clamp((climb-pitch)/30,-1,1)))
    }}
    return nil
}
func main() { if err:=cy.Run(&starter{});err!=nil {panic(err)} }
