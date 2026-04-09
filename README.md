# README

## Artemis II-Style Free-Return Trajectory Demo

This MATLAB script simulates a simplified **Earth–Moon free-return mission** inspired by an Artemis II-style trajectory. The spacecraft begins in a circular low Earth parking orbit, completes one Earth orbit, performs a **trans-lunar injection (TLI)** burn, makes a **small trajectory correction**, flies past the **far side of the Moon**, returns toward Earth, and performs **one final burn** to circularize back into Earth orbit.

## Features

- 2D Earth-centered trajectory simulation
- Earth fixed at the origin
- Moon modeled on a circular orbit around Earth
- One full parking orbit before TLI
- One major TLI burn
- One small midcourse correction burn
- Targeted far-side lunar flyby
- Direct Earth return
- One-burn Earth-orbit circularization on return
- Static plots of the trajectory and distance histories
- Animated GIF export of the mission profile

## How It Works

The script uses a two-stage targeting process:

### 1. Coarse Seed Search

A grid search sweeps over:

- initial parking-orbit phase angle
- Moon phase at TLI
- TLI burn magnitude
- TLI burn flight-path angle

It evaluates each candidate and keeps the best feasible seed for refinement.

### 2. Refinement

The best seed is refined using `fminsearch` and a bounded objective function. The optimization seeks a trajectory that:

- hits the desired lunar flyby altitude,
- returns near the original Earth orbit radius,
- approaches Earth inbound,
- passes on the far side of the Moon,
- uses a small correction burn.

## Mission Sequence

1. Start in a 300 km circular Earth parking orbit  
2. Coast for one full Earth orbit  
3. Apply TLI burn  
4. Coast toward the Moon  
5. Apply one small correction burn  
6. Perform lunar flyby  
7. Return toward Earth  
8. Circularize into Earth orbit at first inbound crossing of the target radius, or at the first post-flyby local minimum radius if needed.

## Model Assumptions

This is a simplified educational/demo model, not a full mission design tool.

Assumptions include:

- planar 2D motion only,
- Earth fixed in place,
- Moon on a circular orbit,
- only Earth and Moon gravity included,
- impulsive burns,
- no Sun, no non-spherical gravity, no atmospheric drag, and no 3D effects.

## Inputs / Tunable Parameters

Key parameters near the top of the script include:

### Physical constants

- `p.muE` — Earth gravitational parameter
- `p.muM` — Moon gravitational parameter
- `p.Re` — Earth radius
- `p.Rm` — Moon radius
- `p.Dem` — Earth–Moon distance

### Parking orbit

- `p.altitude0` — initial parking orbit altitude
- `p.r0` — parking orbit radius
- `p.vcirc` — circular speed at parking orbit
- `p.Torbit` — parking orbit period
- `p.nParkingOrbits` — number of Earth orbits before TLI

### Mission timing

- `p.tfinal` — simulation end time
- `p.nominalCorrDays` — nominal correction timing
- `p.nPostCaptureOrbits` — extra post-capture orbits shown
- `p.tPostCapture` — post-capture coast duration

### Targeting goals

- `p.targetFlybyAlt` — desired lunar flyby altitude
- `p.targetReturnRad` — desired Earth return radius
- `p.maxCorrDV` — preferred upper scale for correction burn size

### GIF controls

- `makeGIF` — enable/disable GIF creation
- `gifName` — output GIF filename
- `gifDelay` — frame delay
- `gifSkip` — frame subsampling
- `showFullTrail` — show full trajectory or short tail
- `trailLength` — tail length when not showing full trail

## Outputs

When run, the script produces:

- console output with optimized trajectory parameters and mission summary,
- a static Earth-centered trajectory figure,
- a figure showing distance to Earth and distance to Moon over time,
- an animated GIF of the mission.

Reported summary values include:

- launch angle
- Moon phase at TLI
- TLI magnitude and angle
- correction timing and vector
- flyby altitude
- Earth return radius
- Earth return radial velocity
- far-side metric
- final circularization burn
- final circularized orbit radius

## Main Functions

### `simulate_free_return(x, p)`

Propagates the spacecraft through:

- initial Earth parking orbit coast,
- TLI burn,
- coast to correction,
- correction burn,
- final coast to the end of the free-return arc.

It also computes flyby altitude, return radius, far-side metric, and event indices.

### `add_earth_recapture(sol, p)`

Finds a suitable Earth-return point and applies one burn to circularize into Earth orbit, then propagates the final circular coast.

### `objective_from_solution(x, sol, p)`

Builds the optimization cost function based on flyby altitude, Earth return radius, radial return velocity, far-side passage quality, and correction burn size.

### `rhs(t, y, p, moonPhase0)`

Defines the equations of motion under Earth and Moon gravity.

### `moon_position(t, p, moonPhase0)` / `moon_state(t, p, moonPhase0)`

Return the Moon’s position and velocity in the 2D Earth-centered frame.