(define (domain robot_marker_scanning)

  (:requirements :strips :typing :fluents :universal-preconditions)

  (:types
    waypoint robot
  )

  (:predicates
    (robot_at ?r - robot ?wp - waypoint)   ; Robot is at a specific waypoint
    (visited ?wp - waypoint)              ; Waypoint has been visited
    (visited_and_scanned ?wp - waypoint)  ; Waypoint has been visited and its marker scanned
  )

  ;; Move to a waypoint
  (:action move
    :parameters (?r - robot ?from ?to - waypoint)
    :precondition (and
      (robot_at ?r ?from)
    )
    :effect (and
      (robot_at ?r ?to)
      (visited ?to)
      (not (robot_at ?r ?from))
    )
  )

  ;; Scan the marker at a waypoint
  (:action scan_marker
    :parameters (?r - robot ?wp - waypoint)
    :precondition (and
      (robot_at ?r ?wp)
      (not (visited_and_scanned ?wp))
      (visited ?wp)       ; Waypoint must be visited before scanning
    )
    :effect (and
      (visited_and_scanned ?wp)
    )
  )
)
