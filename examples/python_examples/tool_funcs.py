def get_repeating_items_in_set(ls: list) -> set:
    """
    Args:
        ls (list): list of items that may have repeating items

    Returns:
        set: set of items that have appeared at least once
    """
    visited = set()
    revisited = set()
    for item in ls:
        if item not in visited:
            visited.add(item)
        else: 
            revisited.add(item)
    return revisited


def check_calib_file_zeroed_out() -> bool:
    with open(DEFAULT_CALIB_CONFIG_PATH, "r") as calib_yaml_file:
        current_calib = yaml.safe_load(calib_yaml_file)
    rpy = np.fromstring(
        current_calib["tf_calibration"]["torso"]["rpy"],
        dtype=float,
        sep=" ")
    xyz = np.fromstring(
        current_calib["tf_calibration"]["torso"]["xyz"],
        dtype=float,
        sep=" ")
    zeros_3d = np.zeros(3)
    return np.allclose(rpy, zeros_3d) and np.allclose(xyz, zeros_3d)

