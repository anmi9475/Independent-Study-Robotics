(define
	(problem blocks)
	(:domain blocks)
	(:objects
		redBlock yellowBlock blueBlock loc-a loc-b loc-c - object
	)
	(:init (Block redBlock) (Block blueBlock) (Block yellowBlock) (on redBlock yellowBlock) (on yellowBlock loc-a) (on blueBlock loc-c) (fixed loc-a) (fixed loc-b) (fixed loc-c) (clear redBlock) (clear blueBlock) (clear loc-b))
	(:goal (on blueBlock yellowBlock))
)
