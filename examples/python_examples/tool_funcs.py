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
