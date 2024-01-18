(define
	(problem blocks)
	(:domain blocks)
	(:objects
		redBlock yellowBlock blueBlock loc-a loc-b loc-c - object
	)
	(:init (Block redBlock) (Block blueBlock) (Block yellowBlock) (on yellowBlock loc-a) (on blueBlock loc-c) (on redBlock loc-b) (fixed loc-a) (fixed loc-b) (fixed loc-c) (clear yellowBlock) (clear blueBlock) (clear redBlock))
	(:goal (on blueBlock yellowBlock))
)
