(define (domain domain_name)

	(:requirements :strips :typing :adl :fluents :durative-actions :negative-preconditions)

	(:types 
	    robot 
	    waypoint 
	    marker
		recharge_zone
	)

	(:predicates
	    (at ?r - robot ?w - waypoint)         ;; Il robot è in un waypoint
	    (detected ?r - robot ?w - waypoint ?m - marker) ;; Il robot ha rilevato un marker nel waypoint
	    (done ?r - robot ?w1 - waypoint ?w2 - waypoint ?w3 - waypoint ?w4 - waypoint) ;; Il robot ha completato la missione
	    (connected ?wp1 - waypoint ?wp2 - waypoint)
		(in_charge ?r - robot ?w1 - waypoint ?w2 - waypoint ?w3 - waypoint ?w4 - waypoint ?cz - recharge_zone) 		;; Il robot è in una zona di ricarica
		(connected_path ?w - waypoint ?cz - recharge_zone) ;; Il robot segue una routine di ricarica
		(charged ?r - robot ?w1 - waypoint ?w2 - waypoint ?w3 - waypoint ?w4 - waypoint ?cz - recharge_zone) ;; Il robot è carico
	)

	; Azione: Spostarsi da un waypoint a un altro
	(:durative-action move
	    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
	    :duration ( = ?duration 10)
	    :condition (and (at start(connected ?wp1 ?wp2)) (at start(at ?r ?wp1)))
	    :effect (and (at start(not(at ?r ?wp1))) (at end(at ?r ?wp2)))
	)

	; Azione: Rilevare un marker in un waypoint
	(:durative-action detect
	    :parameters (?r - robot ?w - waypoint ?m - marker)
	    :duration (= ?duration 10)
	    :condition (and (at start (at ?r ?w)))            ;; Il robot deve trovarsi nel waypoint
	    :effect (and (at end (detected ?r ?w ?m)))        ;; Il marker viene rilevato
	)

	; Azione: Completare la missione
	(:durative-action move_to_lowest_id
		:parameters (?r - robot ?w1 - waypoint ?w2 - waypoint ?w3 - waypoint ?w4 - waypoint ?m1 - marker)
		:duration (= ?duration 10)
		:condition (and 
			(at start (detected ?r ?w1 ?m1))   ;; Marker m1 deve essere rilevato nel waypoint w1
			(at start (detected ?r ?w2 ?m1))   ;; Marker m1 deve essere rilevato nel waypoint w2
			(at start (detected ?r ?w3 ?m1))   ;; Marker m1 deve essere rilevato nel waypoint w3
			(at start (detected ?r ?w4 ?m1))   ;; Marker m1 deve essere rilevato nel waypoint w4
		)
		:effect (and 
			(at end (done ?r ?w1 ?w2 ?w3 ?w4))							;; Il robot ha completato la missione
		)
	)

	; Azione: Robot che va nella zona di ricarica
	(:durative-action move_to_recharge_zone
	    :parameters (?r - robot ?w1 - waypoint ?w2 - waypoint ?w3 - waypoint ?w4 - waypoint ?m - marker ?cz - recharge_zone)
	    :duration (= ?duration 10)
		:condition (and (at start (done ?r ?w1 ?w2 ?w3 ?w4))
						(at start (connected_path ?w1 ?cz)) (at start (connected_path ?w2 ?cz))
						(at start (connected_path ?w3 ?cz)) (at start (connected_path ?w4 ?cz))) 			;; Il robot deve aver completato la missione
	    :effect (and (at end (in_charge ?r ?w1 ?w2 ?w3 ?w4 ?cz)))									;; Il robot rimane nella zona di ricarica
	)

	; Azione: Robot in carica
	(:durative-action charge_action
	    :parameters (?r - robot ?w1 - waypoint ?w2 - waypoint ?w3 - waypoint ?w4 - waypoint ?cz - recharge_zone)
	    :duration (= ?duration 10)
		:condition (and (at start (in_charge ?r ?w1 ?w2 ?w3 ?w4 ?cz)))					;; Il robot deve essere nella zona di ricarica
	    :effect (and (at end (charged ?r ?w1 ?w2 ?w3 ?w4 ?cz))) 	;; Il robot esce dalla zona di ricarica
	)
)