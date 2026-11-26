def run(gripper, goal):
    """Fake stow. Real implementation must verify tool insertion and readiness."""
    gripper.command(0.0, 20.0)
    _, gripped = gripper.status()
    gripper.command(0.1, 0.0)
    return gripped
