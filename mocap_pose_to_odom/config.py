from dataclasses import dataclass

@dataclass
class MocapCfg:
    publish_frequency: float = 10 # hz
    frame: str = 'world'
    pose_buffer_n_arg: int = 10