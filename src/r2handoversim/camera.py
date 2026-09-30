"""Choose a review camera that keeps receiver vertices clear of asset bounds."""
import numpy as np


def visibility(eye, targets, bounds):
    targets=np.asarray(targets,dtype=float);bounds=np.asarray(bounds,dtype=float)
    if not len(bounds): return 1.
    direction=targets-np.asarray(eye)
    parallel=np.abs(direction)<1e-12
    denominator=np.where(parallel,1.,direction)
    a=(bounds[None,:,0,:]-eye)/denominator[:,None,:]
    b=(bounds[None,:,1,:]-eye)/denominator[:,None,:]
    lo=np.minimum(a,b);hi=np.maximum(a,b)
    inside=(eye>=bounds[:,0,:]) & (eye<=bounds[:,1,:])
    lo=np.where(parallel[:,None,:],np.where(inside[None],-np.inf,np.inf),lo)
    hi=np.where(parallel[:,None,:],np.where(inside[None],np.inf,-np.inf),hi)
    near=lo.max(2);far=hi.min(2)
    blocked=(far>=np.maximum(near,0.)) & (near<.995)
    return float(np.mean(~blocked.any(1)))


def choose(hand_vertices,target,bounds_by_pose,scale=1.):
    hand=np.asarray(hand_vertices)[::max(1,len(hand_vertices)//64)]
    target=np.asarray(target)
    base_angle=np.arctan2(target[1],target[0])  # receiver side, looking toward robot
    offsets=[0.,np.pi/6,-np.pi/6,np.pi/3,-np.pi/3,np.pi/2,-np.pi/2,2*np.pi/3,-2*np.pi/3,np.pi]
    best=None
    for height in (.7,1.1):
        for offset in offsets:
            angle=base_angle+offset
            eye=target+scale*np.array([np.cos(angle),np.sin(angle),height])
            visible=[visibility(eye,hand,bounds) for bounds in bounds_by_pose]
            score=min(visible)+.1*np.mean(visible)
            if best is None or score>best[0]+1e-9: best=(score,eye,visible)
    return best[1],{'sampled_pose_visibility':best[2],
                   'visibility_rule':'Hand vertex rays against original asset world AABBs; camera heuristic only'}
