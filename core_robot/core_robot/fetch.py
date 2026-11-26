def run(gripper, goal):
    """Fake handoff. Real implementation must confirm grip and release at dest_id."""
    gripper.command(0.0, 20.0)
    _, gripped = gripper.status()
    gripper.command(0.1, 0.0)
    return gripped
