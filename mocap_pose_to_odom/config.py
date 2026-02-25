from dataclasses import dataclass

@dataclass
class MocapCfg:
    frame: str = 'world'
    buffer_size: int = 10

    # Low pass filter params
    mocap_rate_hz = 100.0
    lpf_order = 3
    lpf_cutoff_hz = 1.0