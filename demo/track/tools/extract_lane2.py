import cv2,numpy as np
from scipy.interpolate import splprep,splev
from scipy.ndimage import median_filter
lab=np.load('lab.npy');lab2=np.load('lab2.npy')
X0,YT,YB,XR=338.5,255.0,2205.0,3234.0
sx=16000/(XR-X0); sy=11000/(YB-YT); f=sy/sx   # stretch y to make isotropic (mm/px = sx)
H,W=lab.shape; Hn=int(round(H*f))
def iso(m): return cv2.resize(m.astype(np.uint8),(W,Hn),interpolation=cv2.INTER_NEAREST)>0
outer=iso(np.isin(lab,[1,3])); cen=iso(lab2==32)
Do=cv2.distanceTransform((~outer).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
Dc=cv2.distanceTransform((~cen).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
off=450/sx
inner=iso(lab==2)
Di=cv2.distanceTransform((~inner).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
road=1850/sx
lvl=(np.abs(Do-off)<0.8)&(Di<road)
ys,xs=np.nonzero(lvl)
P=np.c_[(xs-X0)*sx,(YB*f-ys)*sx]/1000.0
c=np.array([8.0,5.5]); d=P-c; ang=np.arctan2(d[:,1],d[:,0]); rad=np.hypot(*d.T)
nb=1440; bins=((ang+np.pi)/(2*np.pi)*nb).astype(int)%nb
R=np.full(nb,np.nan)
for b in range(nb):
    m=bins==b
    if m.any(): R[b]=np.median(rad[m])
th=(np.arange(nb)+0.5)/nb*2*np.pi-np.pi
ok=~np.isnan(R); print('empty bins',(~ok).sum())
Ri=np.interp(th,th[ok],R[ok],period=2*np.pi); Rs=median_filter(Ri,size=9,mode='wrap')
Q=c+np.c_[np.cos(th),np.sin(th)]*Rs[:,None]
tck,_=splprep([np.r_[Q[:,0],Q[0,0]],np.r_[Q[:,1],Q[0,1]]],s=0.01,per=1)
L=np.c_[splev(np.linspace(0,1,8000,endpoint=False),tck)].T
fin_y=(YB-1171.5)*sy/1000+0.05   # 앞범퍼 = 10 cm 선의 차 쪽(+y) 가장자리
i0=np.argmin(np.hypot(L[:,0]-1.2,L[:,1]-fin_y))
if L[(i0+1)%len(L),1]>L[i0,1]: L=L[::-1]; i0=len(L)-1-i0
L=np.roll(L,-i0,axis=0)
seg=np.hypot(*np.diff(np.r_[L,L[:1]],axis=0).T); s=np.r_[0,np.cumsum(seg)[:-1]]
S=np.arange(0,seg.sum(),0.05); Lx=np.interp(S,s,L[:,0]); Ly=np.interp(S,s,L[:,1])
hd=np.unwrap(np.arctan2(np.gradient(Ly),np.gradient(Lx))); k=np.gradient(hd)/0.05
np.savetxt('lane2_centerline.csv',np.c_[S,Lx,Ly,np.degrees(np.arctan2(np.sin(hd),np.cos(hd))),k],delimiter=',',header='s_m,x_m,y_m,heading_deg,curvature_1pm',comments='',fmt='%.4f')
print('loop %.2f m start (%.3f,%.3f) hd %.1f'%(seg.sum(),Lx[0],Ly[0],np.degrees(hd[0])))
r=1/np.maximum(np.abs(k),1e-9); print('min R %.2f m at s=%.2f'%(r.min(),S[r.argmin()]))
# overlay on original ccw image
im=cv2.imread('ccw.png')
px=np.c_[Lx*1000/sx+X0, YB-Ly*1000/sy].astype(np.int32)
cv2.polylines(im,[px],True,(0,0,230),5,cv2.LINE_AA)
for j in range(0,len(px),60):
    a=px[j]; b=px[min(j+8,len(px)-1)]
    cv2.arrowedLine(im,tuple(map(int,a)),tuple(map(int,a+(b-a)*1.5)),(0,0,230),5,tipLength=0.6)
p0=px[0]; cv2.circle(im,tuple(map(int,p0)),16,(200,0,200),-1)
cv2.putText(im,'START: front bumper on 10cm line, lane 2',(int(p0[0])+30,int(p0[1])-25),cv2.FONT_HERSHEY_SIMPLEX,1.3,(200,0,200),3)
cv2.imwrite('/home/lee/demo/track/start_pose_check.png',cv2.resize(im,(1755,1242)))
