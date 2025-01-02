(define (problem robot_marker_scanning_problem)
  (:domain robot_marker_scanning)

  ;; Define the objects in the problem
  (:objects
    robot1 - robot
    initpos wp1 wp2 wp3 wp4 - waypoint
  )

  ;; Define the initial state
  (:init
    (robot_at robot1 initpos)        ; Robot starts at the initial position
    (not (visited wp1))              ; Initially, all waypoints are not visited
    (not (visited wp2))
    (not (visited wp3))
    (not (visited wp4))
    
    (not (visited_and_scanned wp1))  ; Initially, all waypoints are not scanned
    (not (visited_and_scanned wp2))
    (not (visited_and_scanned wp3))
    (not (visited_and_scanned wp4))
  )

  ;; Define the goal state
  (:goal
    (and
      (visited_and_scanned wp1)      ; All waypoints must be visited and scanned
      (visited_and_scanned wp2)
      (visited_and_scanned wp3)
      (visited_and_scanned wp4)
    )
  )
)
