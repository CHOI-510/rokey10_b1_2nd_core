def require_fake(mode):
    if mode != 'fake':
        raise ValueError('P0-a supports mode=fake only')
