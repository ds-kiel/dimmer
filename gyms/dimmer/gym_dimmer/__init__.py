import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='CentralizedControl-v0',
    entry_point='gym_dimmer.envs.centralized:CentralizedControl',
)

