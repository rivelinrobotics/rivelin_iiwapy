def clamp(name: str, val: float, _min: float, _max: float) -> float:
    if val < _min or val > _max:
        raise ValueError(f"{name} must be greater than {_min} and less than {_max}")

    return val


class ControllerError(Exception):
    pass
