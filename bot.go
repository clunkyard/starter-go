// A robot on the Clunkyard core's Go binding. The core owns tracking,
// geometry and protocol handling; this strategy chooses the tactics.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range)
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   tracker.configure(smoothing forget_ticks cue_ticks)
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
type starter struct { freeFlight bool }
func value[T any](out T, err error) T { if err != nil { panic(err) }; return out }
func check(err error) { if err != nil { panic(err) } }
func steer(r *cy.Robot, towards, current float64) float64 { return value(r.Angles.Clamp(value(r.Angles.AngleDiff(towards,current))/45,-1,1)) }
func drive(r *cy.Robot,throttle,turn,pitch float64) { check(r.Drive.Set(cy.DriveSet{Throttle:cy.Num(throttle),Turn:cy.Num(turn),PitchTurn:cy.Num(pitch)})) }
func scan(r *cy.Robot,heading,arc,pitch float64) { check(r.Scanner.Set(cy.ScannerSet{Heading:cy.Num(heading),Arc:cy.Num(arc),Pitch:cy.Num(pitch)})) }
func (s *starter) Start(r *cy.Robot) error { s.freeFlight=value(r.Match.Is3d())&&value(r.Arena.Gravity()).Z==0; return nil }
func (s *starter) Turn(r *cy.Robot) error {
    position,heading,pitch,tick:=value(r.Me.Position()),value(r.Me.Heading()),value(r.Me.Pitch()),value(r.Clock.Tick())
    check(r.Drive.Forces(cy.DriveForces{Brake:cy.Num(0.2)}))
    if target:=value(r.Sensors.Nearest());target!=nil {
        throttle:=0.2;if target.Range>350 {throttle=0.7} else if target.Range<200 {throttle=-0.4}
        drive(r,throttle,steer(r,target.Bearing,heading),steer(r,target.Elevation,pitch));scan(r,target.Bearing,20,target.Elevation)
        power:=value(r.Gun.PowerForFlightTime(target.Range,1));aim:=value(r.Gun.Lead(*target,power))
        towards,up:=target.Bearing,target.Elevation;if aim!=nil {towards,up=aim.Heading,aim.Pitch}
        check(r.Gun.Aim(cy.GunAim{Heading:cy.Num(towards),Pitch:cy.Num(up)}));now:=value(r.Gun.AimNow())
        if math.Abs(value(r.Angles.AngleDiff(towards,now.Heading)))<3&&math.Abs(up-now.Pitch)<3&&value(r.Gun.CanFire(power,10)) {check(r.Gun.Fire(cy.GunFire{Power:cy.Num(power)}))}
        check(r.Status.Set(cy.StatusSet{Text:cy.Text("TRACK"),Value:cy.Num(target.Range)}))
    } else if cue:=value(r.Tracker.Cue());cue!=nil {
        bearing,distance:=value(r.Angles.BearingTo(position,cue.Position)),value(r.Angles.Distance(position,cue.Position))
        elevation:=0.0;if s.freeFlight {elevation=value(r.Angles.ElevationTo(position,cue.Position))}
        error:=math.Min(cue.BearingError,30);across:=float64(tick%5-2)/2*error;up:=0.0;if s.freeFlight {up=float64((tick/5)%3-1)*error/2}
        drive(r,0.7,steer(r,bearing,heading),steer(r,elevation,pitch));scan(r,bearing+across,value(r.Scanner.ArcForReach(distance+100)),elevation+up)
        check(r.Status.Set(cy.StatusSet{Text:cy.Text("CUE"),Value:cy.Num(distance)}))
    } else {
        drive(r,0.5,0.33,steer(r,0,pitch));up:=0.0;if s.freeFlight {up=float64((tick/10)%3-1)*35}
        scan(r,value(r.Angles.Normalize(value(r.Scanner.Last()).Heading+36)),40,up);check(r.Status.Set(cy.StatusSet{Text:cy.Text("SEARCH")}))
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
