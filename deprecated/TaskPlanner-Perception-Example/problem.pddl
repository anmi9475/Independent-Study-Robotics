(define
	(problem blocks)
	(:domain blocks)
	(:objects
		redBlock yellowBlock blueBlock loc-a loc-b loc-c - object
	)
	(:init (Block redBlock) (Block blueBlock) (Block yellowBlock) (on yellowBlock redBlock) (on redBlock loc-a) (on blueBlock loc-c) (fixed loc-a) (fixed loc-b) (fixed loc-c) (clear yellowBlock) (clear blueBlock) (clear loc-b))
	(:goal (and (on redBlock loc-c) (on yellowBlock redBlock) (on blueBlock yellowBlock)))
)
