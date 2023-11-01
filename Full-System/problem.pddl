(define
	(problem blocks)
	(:domain blocks)
	(:objects
		redBlock yellowBlock blueBlock loc-a loc-b loc-c - object
	)
	(:init (Block redBlock) (Block blueBlock) (Block yellowBlock) (on yellowBlock blueBlock) (on redBlock loc-a) (on blueBlock loc-c) (fixed loc-a) (fixed loc-b) (fixed loc-c) (clear yellowBlock) (clear redBlock) (clear loc-b))
	(:goal (on redBlock yellowBlock))
)
