(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
    waypoint robot counter waypointf
  );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates
    (robot_at ?r - robot ?wp - waypoint) ; Robot is at a specific waypoint
    (visited ?wp - waypoint)            ; Waypoint has been visited
    (marker_scanned ?wp - waypoint)     ; Marker at the waypoint has been scanned
    (target_reached ?wp - waypointf)     ; Target waypoint has been reached
    (final_wp ?wp - waypointf)
    (not_visited ?wp - waypoint)
);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:action move
    :parameters (?r - robot ?from ?to - waypoint)
    :precondition (and
      (robot_at ?r ?from)
      (not_visited ?to)
    )
    :effect (and
      (robot_at ?r ?to)
      (not (robot_at ?r ?from))
    )
  )


(:action scan_marker
    :parameters (?r - robot ?wp - waypoint )
    :precondition (and
      (robot_at ?r ?wp)
      )
    
    :effect (and
      (marker_scanned ?wp)
      (visited ?wp)
      (not(not_visited ?wp))
     
    )
  )



(:action move_to_target
    :parameters (?r - robot ?from - waypoint ?wp - waypointf)
    :precondition (and
      (robot_at ?r ?from)
      (final_wp ?wp)
      (forall (?w - waypoint)
        (marker_scanned ?w) ; Ensure all waypoints are scanned
      )
    )
    :effect (and
      (target_reached ?wp) ; Mark the target waypoint as reached
      (not (robot_at ?r ?from))
    )



)








);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;