from init import lift, pressure

lift_speed=600
async def reset_lift():
    lift.run(lift_speed)
    while not await pressure.pressed(force=.5):
      pass
    print("button pressed. stopping lift")
    lift.stop()
    print("moving to zero")
    await lift.run_angle(-lift_speed, 90)
    print("lift at zero")
    lift.reset_angle(0)
    
async def lift_to_height(height_to_go_to):
    print("lifting to height:", height_to_go_to)
    await lift.run_target(lift_speed, -height_to_go_to)
