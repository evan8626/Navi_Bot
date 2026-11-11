import math, sys
import numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot\tests")
import scan_matching_test as T

def icp(cur, ref, guess=(0,0,0), iters=200, trim=None, p2l=False):
    ref=np.asarray(ref,float); cur=np.asarray(cur,float); tree=cKDTree(ref)
    # normals for point-to-line
    _, nn = tree.query(ref, k=5)
    normals=[]
    for i in range(len(ref)):
        P=ref[nn[i]]-ref[nn[i]].mean(0); w,v=np.linalg.eigh(P.T@P); normals.append(v[:,0])
    normals=np.array(normals)
    x,y,th=guess
    for _ in range(iters):
        c,s=math.cos(th),math.sin(th); Rm=np.array([[c,-s],[s,c]])
        moved=cur@Rm.T+[x,y]; d,idx=tree.query(moved)
        keep=np.ones(len(d),bool) if trim is None else d<=np.percentile(d,trim)
        if p2l:
            n=normals[idx[keep]]; q=ref[idx[keep]]; m=moved[keep]
            J=np.column_stack([n[:,0],n[:,1], n[:,1]*m[:,0]-n[:,0]*m[:,1]])
            r=np.sum(n*(q-m),1)
            dx,dy,dth=np.linalg.lstsq(J,r,rcond=None)[0]
            # compose increment
            c2,s2=math.cos(dth),math.sin(dth)
            x,y = c2*x - s2*y + dx, s2*x + c2*y + dy; th += dth
        else:
            A=cur[keep]; B=ref[idx[keep]]; ma,mb=A.mean(0),B.mean(0)
            U,S,Vt=np.linalg.svd((A-ma).T@(B-mb)); R=Vt.T@U.T
            t=mb-R@ma; x,y,th=t[0],t[1],math.atan2(R[1,0],R[0,0])
    return x,y,th
room=T.synthetic_room()
for tt in [(0.2,-0.1,0.0),(0,0,math.radians(8)),(0.15,0.1,math.radians(5)),(0.12,-0.08,math.radians(4))]:
    cur=T.make_current(room,tt)
    for kw in [dict(), dict(p2l=True)]:
        g=icp(cur,room,**kw); print(tt, kw, tuple(round(float(v),4) for v in g), T.transform_close(g,tt,0.02,0.5)[0])
tt=(0.15,0.05,math.radians(6)); rng=np.random.default_rng(42)
cur=[(x+rng.normal(0,0.01),y+rng.normal(0,0.01)) for x,y in T.make_current(room,tt)]
for kw in [dict(), dict(p2l=True)]:
    g=icp(cur,room,**kw); print('noisy',kw,tuple(round(float(v),4) for v in g), T.transform_close(g,tt,0.05,2.0)[0])
tt=(0.1,-0.05,math.radians(4)); full=T.make_current(room,tt); cur=full[:int(len(full)*0.6)]
for kw in [dict(), dict(p2l=True)]:
    g=icp(cur,room,**kw); print('partial',kw,tuple(round(float(v),4) for v in g), T.transform_close(g,tt,0.06,3.0)[0])
