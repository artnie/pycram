import time
from . import plan_failures
from .designators.action_designator import ActionDesignatorDescription
from .bullet_world import BulletWorld
from .bullet_world_reasoning import contact


class FailureHandling:
    """
    My amazing failure handling description for aaaaaall the failures to be handled
    """

    def __init__(self):
        ...


class WithRetries(FailureHandling):
    """
    The strategy to re-execute an action designator
    """

    def __init__(self, action_description: ActionDesignatorDescription, retries=10):
        super().__init__()
        self.action_description = action_description
        # self.action_generator = iter(action_description)
        self.retries = retries

    def perform(self):
        # try:
        #     next(self.action_generator).ground().perform()
        # except pycram.plan_failures.NavigationGoalInCollision as failure:
        #     time.sleep(0.5)
        #     if self.retries > 0:
        #         self.retries -= 1
        #         print('oopsi poopsi', failure)
        #         self.perform()
        #     else:
        #         raise failure

        # for action in self.action_generator:
        #     time.sleep(0.5)
        #     try:
        #         action.ground().perform()
        #         break
        #     except pycram.plan_failures.PlanFailure as failure:
        #         print('oopsi poopsi', failure)

        for action in iter(self.action_description):
            # robot = BulletWorld.robot
            # env = BulletWorld.current_bullet_world.get_objects_by_name(name="kitchen")[0]

            time.sleep(0.5)
            try:
                action.ground().perform()
                print("I did it!!!")
                # raise plan_failures.NavigationGoalInCollision("After the navigation")
                break
            except plan_failures.PlanFailure as failure:
                print('plan failure', failure)
                self.retries -= 1
                if self.retries <= 0:
                    raise failure
            # except StopIteration as no_solutions_failure:
            #     print('no solutions left', no_solutions_failure)
            #     raise
