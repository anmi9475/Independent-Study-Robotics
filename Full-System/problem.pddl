(define
	(problem blocks)
	(:domain blocks)
	(:objects
		redBlock yellowBlock blueBlock loc-a loc-b loc-c - object
	)
	(:init (Block redBlock) (Block blueBlock) (Block yellowBlock) (on blueBlock yellowBlock) (on yellowBlock loc-a) (on redBlock loc-c) (fixed loc-a) (fixed loc-b) (fixed loc-c) (clear blueBlock) (clear redBlock) (clear loc-b))
	(:goal (and (on blueBlock yellowBlock) (on redBlock yellowBlock)))
)
