(define (problem pfile21)
(:domain TPP-MetricTime)
(:objects
	market1 market2 market3 market4 market5 market6 - market
	depot1 depot2 depot3 - depot
	truck1 truck2 truck3 truck4 truck5 truck6 - truck
	goods1 goods2 goods3 goods4 goods5 goods6 goods7 goods8 goods9 goods10 goods11 - goods)

(:init
	(= (price goods1 market1) 49)
	(= (ready-to-load goods1 market1) 0)
	(= (on-sale goods1 market1) 17)
	(= (ready-to-load goods1 market2) 0)
	(= (on-sale goods1 market2) 0)
	(= (ready-to-load goods1 market3) 0)
	(= (on-sale goods1 market3) 0)
	(= (price goods1 market4) 15)
	(= (ready-to-load goods1 market4) 0)
	(= (on-sale goods1 market4) 16)
	(= (ready-to-load goods1 market5) 0)
	(= (on-sale goods1 market5) 0)
	(= (price goods1 market6) 30)
	(= (ready-to-load goods1 market6) 0)
	(= (on-sale goods1 market6) 7)
	(= (price goods2 market1) 29)
	(= (ready-to-load goods2 market1) 0)
	(= (on-sale goods2 market1) 14)
	(= (price goods2 market2) 14)
	(= (ready-to-load goods2 market2) 0)
	(= (on-sale goods2 market2) 4)
	(= (ready-to-load goods2 market3) 0)
	(= (on-sale goods2 market3) 0)
	(= (price goods2 market4) 18)
	(= (ready-to-load goods2 market4) 0)
	(= (on-sale goods2 market4) 17)
	(= (price goods2 market5) 17)
	(= (ready-to-load goods2 market5) 0)
	(= (on-sale goods2 market5) 3)
	(= (price goods2 market6) 14)
	(= (ready-to-load goods2 market6) 0)
	(= (on-sale goods2 market6) 13)
	(= (ready-to-load goods3 market1) 0)
	(= (on-sale goods3 market1) 0)
	(= (price goods3 market2) 18)
	(= (ready-to-load goods3 market2) 0)
	(= (on-sale goods3 market2) 20)
	(= (ready-to-load goods3 market3) 0)
	(= (on-sale goods3 market3) 0)
	(= (price goods3 market4) 11)
	(= (ready-to-load goods3 market4) 0)
	(= (on-sale goods3 market4) 15)
	(= (ready-to-load goods3 market5) 0)
	(= (on-sale goods3 market5) 0)
	(= (price goods3 market6) 12)
	(= (ready-to-load goods3 market6) 0)
	(= (on-sale goods3 market6) 5)
	(= (ready-to-load goods4 market1) 0)
	(= (on-sale goods4 market1) 0)
	(= (price goods4 market2) 47)
	(= (ready-to-load goods4 market2) 0)
	(= (on-sale goods4 market2) 3)
	(= (price goods4 market3) 24)
	(= (ready-to-load goods4 market3) 0)
	(= (on-sale goods4 market3) 11)
	(= (price goods4 market4) 32)
	(= (ready-to-load goods4 market4) 0)
	(= (on-sale goods4 market4) 16)
	(= (ready-to-load goods4 market5) 0)
	(= (on-sale goods4 market5) 0)
	(= (price goods4 market6) 36)
	(= (ready-to-load goods4 market6) 0)
	(= (on-sale goods4 market6) 1)
	(= (price goods5 market1) 36)
	(= (ready-to-load goods5 market1) 0)
	(= (on-sale goods5 market1) 8)
	(= (ready-to-load goods5 market2) 0)
	(= (on-sale goods5 market2) 0)
	(= (price goods5 market3) 8)
	(= (ready-to-load goods5 market3) 0)
	(= (on-sale goods5 market3) 5)
	(= (ready-to-load goods5 market4) 0)
	(= (on-sale goods5 market4) 0)
	(= (price goods5 market5) 1)
	(= (ready-to-load goods5 market5) 0)
	(= (on-sale goods5 market5) 2)
	(= (price goods5 market6) 13)
	(= (ready-to-load goods5 market6) 0)
	(= (on-sale goods5 market6) 12)
	(= (ready-to-load goods6 market1) 0)
	(= (on-sale goods6 market1) 0)
	(= (price goods6 market2) 3)
	(= (ready-to-load goods6 market2) 0)
	(= (on-sale goods6 market2) 16)
	(= (ready-to-load goods6 market3) 0)
	(= (on-sale goods6 market3) 0)
	(= (ready-to-load goods6 market4) 0)
	(= (on-sale goods6 market4) 0)
	(= (price goods6 market5) 40)
	(= (ready-to-load goods6 market5) 0)
	(= (on-sale goods6 market5) 7)
	(= (price goods6 market6) 27)
	(= (ready-to-load goods6 market6) 0)
	(= (on-sale goods6 market6) 18)
	(= (price goods7 market1) 48)
	(= (ready-to-load goods7 market1) 0)
	(= (on-sale goods7 market1) 2)
	(= (ready-to-load goods7 market2) 0)
	(= (on-sale goods7 market2) 0)
	(= (price goods7 market3) 27)
	(= (ready-to-load goods7 market3) 0)
	(= (on-sale goods7 market3) 15)
	(= (price goods7 market4) 12)
	(= (ready-to-load goods7 market4) 0)
	(= (on-sale goods7 market4) 4)
	(= (price goods7 market5) 3)
	(= (ready-to-load goods7 market5) 0)
	(= (on-sale goods7 market5) 11)
	(= (price goods7 market6) 6)
	(= (ready-to-load goods7 market6) 0)
	(= (on-sale goods7 market6) 4)
	(= (price goods8 market1) 18)
	(= (ready-to-load goods8 market1) 0)
	(= (on-sale goods8 market1) 15)
	(= (ready-to-load goods8 market2) 0)
	(= (on-sale goods8 market2) 0)
	(= (price goods8 market3) 30)
	(= (ready-to-load goods8 market3) 0)
	(= (on-sale goods8 market3) 4)
	(= (price goods8 market4) 30)
	(= (ready-to-load goods8 market4) 0)
	(= (on-sale goods8 market4) 13)
	(= (price goods8 market5) 5)
	(= (ready-to-load goods8 market5) 0)
	(= (on-sale goods8 market5) 5)
	(= (price goods8 market6) 12)
	(= (ready-to-load goods8 market6) 0)
	(= (on-sale goods8 market6) 11)
	(= (ready-to-load goods9 market1) 0)
	(= (on-sale goods9 market1) 0)
	(= (price goods9 market2) 39)
	(= (ready-to-load goods9 market2) 0)
	(= (on-sale goods9 market2) 2)
	(= (price goods9 market3) 24)
	(= (ready-to-load goods9 market3) 0)
	(= (on-sale goods9 market3) 10)
	(= (ready-to-load goods9 market4) 0)
	(= (on-sale goods9 market4) 0)
	(= (price goods9 market5) 33)
	(= (ready-to-load goods9 market5) 0)
	(= (on-sale goods9 market5) 13)
	(= (price goods9 market6) 27)
	(= (ready-to-load goods9 market6) 0)
	(= (on-sale goods9 market6) 17)
	(= (ready-to-load goods10 market1) 0)
	(= (on-sale goods10 market1) 0)
	(= (ready-to-load goods10 market2) 0)
	(= (on-sale goods10 market2) 0)
	(= (ready-to-load goods10 market3) 0)
	(= (on-sale goods10 market3) 0)
	(= (ready-to-load goods10 market4) 0)
	(= (on-sale goods10 market4) 0)
	(= (price goods10 market5) 35)
	(= (ready-to-load goods10 market5) 0)
	(= (on-sale goods10 market5) 5)
	(= (price goods10 market6) 50)
	(= (ready-to-load goods10 market6) 0)
	(= (on-sale goods10 market6) 17)
	(= (price goods11 market1) 4)
	(= (ready-to-load goods11 market1) 0)
	(= (on-sale goods11 market1) 7)
	(= (price goods11 market2) 34)
	(= (ready-to-load goods11 market2) 0)
	(= (on-sale goods11 market2) 6)
	(= (ready-to-load goods11 market3) 0)
	(= (on-sale goods11 market3) 0)
	(= (ready-to-load goods11 market4) 0)
	(= (on-sale goods11 market4) 0)
	(= (price goods11 market5) 36)
	(= (ready-to-load goods11 market5) 0)
	(= (on-sale goods11 market5) 1)
	(= (price goods11 market6) 9)
	(= (ready-to-load goods11 market6) 0)
	(= (on-sale goods11 market6) 11)
	(connected market1 market2)
	(connected market2 market1)
	(= (drive-cost market1 market2) 714.00)
	(= (drive-cost market2 market1) 714.00)
	(= (drive-time market1 market2) 2142.00)
	(= (drive-time market2 market1) 2142.00)
	(connected market1 market3)
	(connected market3 market1)
	(= (drive-cost market1 market3) 220.00)
	(= (drive-cost market3 market1) 220.00)
	(= (drive-time market1 market3) 660.00)
	(= (drive-time market3 market1) 660.00)
	(connected market1 market5)
	(connected market5 market1)
	(= (drive-cost market1 market5) 888.00)
	(= (drive-cost market5 market1) 888.00)
	(= (drive-time market1 market5) 2664.00)
	(= (drive-time market5 market1) 2664.00)
	(connected market1 market6)
	(connected market6 market1)
	(= (drive-cost market1 market6) 937.00)
	(= (drive-cost market6 market1) 937.00)
	(= (drive-time market1 market6) 2811.00)
	(= (drive-time market6 market1) 2811.00)
	(connected market2 market5)
	(connected market5 market2)
	(= (drive-cost market2 market5) 710.00)
	(= (drive-cost market5 market2) 710.00)
	(= (drive-time market2 market5) 2130.00)
	(= (drive-time market5 market2) 2130.00)
	(connected market3 market4)
	(connected market4 market3)
	(= (drive-cost market3 market4) 973.00)
	(= (drive-cost market4 market3) 973.00)
	(= (drive-time market3 market4) 2919.00)
	(= (drive-time market4 market3) 2919.00)
	(connected market3 market5)
	(connected market5 market3)
	(= (drive-cost market3 market5) 870.00)
	(= (drive-cost market5 market3) 870.00)
	(= (drive-time market3 market5) 2610.00)
	(= (drive-time market5 market3) 2610.00)
	(connected market3 market6)
	(connected market6 market3)
	(= (drive-cost market3 market6) 353.00)
	(= (drive-cost market6 market3) 353.00)
	(= (drive-time market3 market6) 1059.00)
	(= (drive-time market6 market3) 1059.00)
	(connected market4 market5)
	(connected market5 market4)
	(= (drive-cost market4 market5) 123.00)
	(= (drive-cost market5 market4) 123.00)
	(= (drive-time market4 market5) 369.00)
	(= (drive-time market5 market4) 369.00)
	(connected market4 market6)
	(connected market6 market4)
	(= (drive-cost market4 market6) 943.00)
	(= (drive-cost market6 market4) 943.00)
	(= (drive-time market4 market6) 2829.00)
	(= (drive-time market6 market4) 2829.00)
	(connected market5 market6)
	(connected market6 market5)
	(= (drive-cost market5 market6) 753.00)
	(= (drive-cost market6 market5) 753.00)
	(= (drive-time market5 market6) 2259.00)
	(= (drive-time market6 market5) 2259.00)
	(connected depot1 market2)
	(connected market2 depot1)
	(= (drive-cost market2 depot1) 381.00)
	(= (drive-cost depot1 market2) 381.00)
	(= (drive-time market2 depot1) 1143.00)
	(= (drive-time depot1 market2) 1143.00)
	(connected depot2 market4)
	(connected market4 depot2)
	(= (drive-cost market4 depot2) 845.00)
	(= (drive-cost depot2 market4) 845.00)
	(= (drive-time market4 depot2) 2535.00)
	(= (drive-time depot2 market4) 2535.00)
	(connected depot3 market2)
	(connected market2 depot3)
	(= (drive-cost market2 depot3) 937.00)
	(= (drive-cost depot3 market2) 937.00)
	(= (drive-time market2 depot3) 2811.00)
	(= (drive-time depot3 market2) 2811.00)
	(= (loaded goods1 truck1) 0)
	(= (loaded goods2 truck1) 0)
	(= (loaded goods3 truck1) 0)
	(= (loaded goods4 truck1) 0)
	(= (loaded goods5 truck1) 0)
	(= (loaded goods6 truck1) 0)
	(= (loaded goods7 truck1) 0)
	(= (loaded goods8 truck1) 0)
	(= (loaded goods9 truck1) 0)
	(= (loaded goods10 truck1) 0)
	(= (loaded goods11 truck1) 0)
	(at truck1 depot3)
	(= (loaded goods1 truck2) 0)
	(= (loaded goods2 truck2) 0)
	(= (loaded goods3 truck2) 0)
	(= (loaded goods4 truck2) 0)
	(= (loaded goods5 truck2) 0)
	(= (loaded goods6 truck2) 0)
	(= (loaded goods7 truck2) 0)
	(= (loaded goods8 truck2) 0)
	(= (loaded goods9 truck2) 0)
	(= (loaded goods10 truck2) 0)
	(= (loaded goods11 truck2) 0)
	(at truck2 depot3)
	(= (loaded goods1 truck3) 0)
	(= (loaded goods2 truck3) 0)
	(= (loaded goods3 truck3) 0)
	(= (loaded goods4 truck3) 0)
	(= (loaded goods5 truck3) 0)
	(= (loaded goods6 truck3) 0)
	(= (loaded goods7 truck3) 0)
	(= (loaded goods8 truck3) 0)
	(= (loaded goods9 truck3) 0)
	(= (loaded goods10 truck3) 0)
	(= (loaded goods11 truck3) 0)
	(at truck3 depot1)
	(= (loaded goods1 truck4) 0)
	(= (loaded goods2 truck4) 0)
	(= (loaded goods3 truck4) 0)
	(= (loaded goods4 truck4) 0)
	(= (loaded goods5 truck4) 0)
	(= (loaded goods6 truck4) 0)
	(= (loaded goods7 truck4) 0)
	(= (loaded goods8 truck4) 0)
	(= (loaded goods9 truck4) 0)
	(= (loaded goods10 truck4) 0)
	(= (loaded goods11 truck4) 0)
	(at truck4 depot3)
	(= (loaded goods1 truck5) 0)
	(= (loaded goods2 truck5) 0)
	(= (loaded goods3 truck5) 0)
	(= (loaded goods4 truck5) 0)
	(= (loaded goods5 truck5) 0)
	(= (loaded goods6 truck5) 0)
	(= (loaded goods7 truck5) 0)
	(= (loaded goods8 truck5) 0)
	(= (loaded goods9 truck5) 0)
	(= (loaded goods10 truck5) 0)
	(= (loaded goods11 truck5) 0)
	(at truck5 depot3)
	(= (loaded goods1 truck6) 0)
	(= (loaded goods2 truck6) 0)
	(= (loaded goods3 truck6) 0)
	(= (loaded goods4 truck6) 0)
	(= (loaded goods5 truck6) 0)
	(= (loaded goods6 truck6) 0)
	(= (loaded goods7 truck6) 0)
	(= (loaded goods8 truck6) 0)
	(= (loaded goods9 truck6) 0)
	(= (loaded goods10 truck6) 0)
	(= (loaded goods11 truck6) 0)
	(at truck6 depot2)
	(= (stored goods1) 0)
	(= (stored goods2) 0)
	(= (stored goods3) 0)
	(= (stored goods4) 0)
	(= (stored goods5) 0)
	(= (stored goods6) 0)
	(= (stored goods7) 0)
	(= (stored goods8) 0)
	(= (stored goods9) 0)
	(= (stored goods10) 0)
	(= (stored goods11) 0)
	(= (total-cost) 0)
	(= (rebate-rate market1) 0.88)
	(= (rebate-rate market2) 0.87)
	(= (rebate-rate market3) 0.86)
	(= (rebate-rate market4) 0.78)
	(= (rebate-rate market5) 0.89)
	(= (rebate-rate market6) 0.82)
	(= (bought goods1) 0)
	(= (bought goods2) 0)
	(= (bought goods3) 0)
	(= (bought goods4) 0)
	(= (bought goods5) 0)
	(= (bought goods6) 0)
	(= (bought goods7) 0)
	(= (bought goods8) 0)
	(= (bought goods9) 0)
	(= (bought goods10) 0)
	(= (bought goods11) 0)
	(= (request goods1) 35)
	(= (request goods2) 1)
	(= (request goods3) 40)
	(= (request goods4) 11)
	(= (request goods5) 17)
	(= (request goods6) 25)
	(= (request goods7) 32)
	(= (request goods8) 45)
	(= (request goods9) 35)
	(= (request goods10) 1)
	(= (request goods11) 19))

(:goal (and
	(>= (stored goods1) (request goods1))
	(>= (stored goods2) (request goods2))
	(>= (stored goods3) (request goods3))
	(>= (stored goods4) (request goods4))
	(>= (stored goods5) (request goods5))
	(>= (stored goods6) (request goods6))
	(>= (stored goods7) (request goods7))
	(>= (stored goods8) (request goods8))
	(>= (stored goods9) (request goods9))
	(>= (stored goods10) (request goods10))
	(>= (stored goods11) (request goods11))))

(:metric minimize (+ (* 1.3 (total-cost))(total-time)))
)