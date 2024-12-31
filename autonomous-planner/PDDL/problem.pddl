(define (problem robot_marker_scanning_problem)
  (:domain robot_marker_scanning)

  ;; Define the objects in the problem
  (:objects
    robot1 - robot
    wp1 wp2 wp3 wp4  - waypoint
    wp5 -waypointf
    counter1 - counter
  )

  ;; Define the initial state
  (:init
    (robot_at robot1 wp1)            ; Robot starts at waypoint wp1
    (final_wp wp5)
    (= (counter_value counter1) 0)   ; Counter starts at 0
    (not (marker_scanned wp1))       ; No markers are scanned initially
    (not (marker_scanned wp2))
    (not (marker_scanned wp3))
    (not (marker_scanned wp4))
    
    (not (target_reached wp5))
    (marker_scanned wp5)
  )

  ;; Define the goal state
  (:goal
    (and
      (target_reached wp5)
)
)
)