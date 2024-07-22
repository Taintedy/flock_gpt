#!/usr/bin/env python3
"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from sdf import box, sphere
import functions.pawn as ex
from cf_show_node import SwarmControllerNode


RATE = 10




TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0


def main():
    # cost = ex.get_pawn()
    cost = [sphere(0.5).translate((0, 0, 2)), box(1).translate((0, 0, 2)), ex.get_pawn()]

    controller = SwarmControllerNode(cost_func=cost, max_vel=0.5)

    all_cfs = controller.all_cfs
    timeHelper = controller.timeHelper
    for cf in all_cfs:
        cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    while timeHelper.time() <= 20:
        controller.show()
        timeHelper.sleepForRate(10)
    
    controller.next_cost()
    while timeHelper.time() <= 40:
        controller.show()
        timeHelper.sleepForRate(10)


    controller.distribute_goals(controller.start_poses)
    while timeHelper.time() <= 80:
        controller.land()
        timeHelper.sleepForRate(10)

    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == "__main__":
    main()
