(define (problem CrewPlanning_1crew_1day_60utilization)
  (:domain CrewPlanningOpl)
  (:objects
    d0 - Day
    d1 - Day
    d2 - Day
    c1 - CrewMember
    e1 - ExerEquipment
    spaceshipFilter - FilterState
    ms1 - MedicalState
    pa1_1 - PayloadAct
    pa1_2 - PayloadAct
    pa1_3 - PayloadAct
    pa1_4 - PayloadAct
  )
  (:init
    (Day_next d0 d1)
    (Day_next d1 d2)
    (Day_initiated d1)
    (CrewMember_currentDay c1 d0)
    (CrewMember_doneSleep c1 d0)
    (CrewMember_available c1)
    (ExerEquipment_unused e1)
  )
  (:goal (and
      (CrewMember_doneSleep c1 d1)
      (Day_initiated d2)
      (MedicalState_finished ms1 d1)
      (PayloadAct_completed pa1_1 d1)
      (PayloadAct_completed pa1_2 d1)
      (PayloadAct_completed pa1_3 d1)
      (PayloadAct_completed pa1_4 d1)))
)
