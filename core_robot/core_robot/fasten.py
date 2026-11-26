from core_common.types import Fastening

RECIPES = {'finish', 'confirm'}


def run(goal, scenario):
    """Fake tightening with distinct confirmation and finishing results."""
    if scenario == 'fasten_failure':
        return Fastening.NOT_SEATED, 0.2, 0.0
    if goal.recipe_id == 'confirm':
        return Fastening.SEATED, 0.3, 0.05
    return Fastening.SEATED, 0.6, 1.25
