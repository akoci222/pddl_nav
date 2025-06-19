(define (problem go-to) (:domain example)
	(:objects
		bot - robot
		catroom sheeproom deerroom emptyroom - room
	)
	(:init
		(in bot emptyroom)
		(cangowest sheeproom catroom)
		(cangowest deerroom emptyroom)

		(cangoeast catroom sheeproom)
		(cangoeast emptyroom deerroom)

		(cangonorth emptyroom catroom)
		(cangonorth deerroom sheeproom)

		(cangosouth sheeproom deerroom)
		(cangosouth catroom emptyroom)

	)
	(:goal
		(in bot emptyroom)
	)
    )