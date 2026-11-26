import numpy as np
from core_vision.bolt_rgb import classify, validate, consensus


def test_combinations():
    c = validate(dict(size=[80,40], radius=10, points={'L':[60,20],'R':[20,20]}))
    for l,r in [('PRESENT','MISSING'),('MISSING','PRESENT'),('UNKNOWN','PRESENT'),('PRESENT','UNKNOWN')]:
        frame = np.full((40,80,3),180,dtype=np.uint8)
        for side,state in [('L',l),('R',r)]:
            x,y=c['points'][side]
            patch=frame[y-10:y+10,x-10:x+10]
            patch[:{'PRESENT':14,'MISSING':2,'UNKNOWN':7}[state]]=30
        assert consensus([classify(frame,c)]*5)==(l,r)
