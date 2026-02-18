from dataclasses import dataclass

@dataclass
class MocapCfg:
    frame: str = 'world'
    buffer_size: int = 10
