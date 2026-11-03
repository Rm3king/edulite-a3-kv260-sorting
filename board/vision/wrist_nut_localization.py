"""Offline single separated nut localization from grayscale annular contours."""
import cv2
import numpy as np


def track_template(gray, template, center_offset, minimum_score=.90, search_center=None, search_radius=60.):
    """Track one reviewed appearance at the same height; not scale/pose invariant.

    center_offset is the reviewed hole center inside the template. The output
    is an image point only; it is never an authorization to move a robot.
    """
    if gray.shape != (400, 640) or template.ndim != 2:
        raise ValueError('Expected grayscale wrist image and template')
    scores = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
    if search_center is not None:
        yy,xx=np.indices(scores.shape)
        valid=(xx+center_offset[0]-search_center[0])**2+(yy+center_offset[1]-search_center[1])**2<=search_radius**2
        scores=np.where(valid,scores,-1.)
    _, score, _, (x, y) = cv2.minMaxLoc(scores)
    if not np.isfinite(score) or score < minimum_score or np.std(template) < 5:
        raise ValueError('Template match rejected')
    if not 0 < x < scores.shape[1]-1 or not 0 < y < scores.shape[0]-1:
        raise ValueError('Match touches image boundary')
    def parabola(a, b, c):
        denominator = float(a)-2*float(b)+float(c)
        return 0. if abs(denominator) < 1e-12 else .5*(float(a)-float(c))/denominator
    dx = parabola(scores[y,x-1], scores[y,x], scores[y,x+1])
    dy = parabola(scores[y-1,x], scores[y,x], scores[y+1,x])
    return dict(center_px=(np.array([x+dx,y+dy])+center_offset).tolist(),
                score=score, method='reviewed_template_subpixel_translation',
                trained_model_used=False)


def locate(gray):
    if gray.shape!=(400,640):raise ValueError('Expected wrist 640x400 frame')
    candidates=[]
    for level in range(35,121,5):
        mask=(gray<level).astype(np.uint8)*255
        contours,hierarchy=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None:continue
        for i,c in enumerate(contours):
            area=cv2.contourArea(c);x,y,w,h=cv2.boundingRect(c);child=hierarchy[0,i,2]
            if not 400<area<20000 or child<0 or x<10 or y<10 or x+w>630 or y+h>390 or not .6<w/h<1.5:continue
            children=[j for j,entry in enumerate(hierarchy[0]) if entry[3]==i]
            child=max(children,key=lambda j:cv2.contourArea(contours[j]))
            hole=contours[child];ha=cv2.contourArea(hole)
            if not .18<ha/area<.7 or len(hole)<5:continue
            (u,v),(a,b),angle=cv2.fitEllipse(hole)
            if min(a,b)/max(a,b)<.65:continue
            m=cv2.moments(c);outer=np.array([m['m10']/m['m00'],m['m01']/m['m00']]);center=np.array([u,v])
            if np.linalg.norm(outer-center)>max(w,h)*.12:continue
            candidates.append(dict(center_px=center.tolist(),outer_center_px=outer.tolist(),threshold=level,area_px=area,hole_area_px=ha,bbox=[x,y,w,h],hole_axes_px=[a,b]))
    if len(candidates)<2:raise ValueError('No annular nut detection')
    points=np.array([v['outer_center_px'] for v in candidates]);outer=np.median(points,axis=0)
    if np.max(np.linalg.norm(points-outer,axis=1))>4.:raise ValueError('Ambiguous outer contours')
    x,y,w,h=candidates[len(candidates)//2]['bbox'];x0=max(0,x-8);y0=max(0,y-8)
    roi=gray[y0:min(400,y+h+8),x0:min(640,x+w+8)]
    circles=cv2.HoughCircles(cv2.GaussianBlur(roi,(5,5),1),cv2.HOUGH_GRADIENT,1,12,param1=70,param2=25,minRadius=int(min(w,h)*.23),maxRadius=int(max(w,h)*.40))
    if circles is None:raise ValueError('Inner rim edge circle not found')
    rings=circles[0].copy();rings[:,:2]+=[x0,y0]
    rings=[r for r in rings if np.linalg.norm(r[:2]-outer)<.12*max(w,h)]
    if not rings:raise ValueError('Rim not concentric with outer contour')
    best=min(rings,key=lambda r:np.linalg.norm(r[:2]-outer));center=best[:2]
    return dict(center_px=center.tolist(),threshold_support=len(candidates),outer_spread_px=float(np.linalg.norm(points-outer,axis=1).max()),bbox=[x,y,w,h],rim_radius_px=float(best[2]),method='annular_contour_with_inner_rim_edge_circle',trained_model_used=False,candidates=candidates)


def locate_near(gray, expected, radius=65.):
    """Near-view inner rim: dark hole may merge with the body at manual exposure.

    Require one complete dark outer component and a concentric edge circle in
    its upper face. Only initialize within the predicted local object window;
    subsequent frames use the same appearance template.
    """
    expected=np.asarray(expected,float);yy,xx=np.indices(gray.shape)
    masked=gray.copy();masked[(xx-expected[0])**2+(yy-expected[1])**2>radius**2]=200
    try:return locate(masked)
    except ValueError:pass
    for level in (25,35,45,55,65):
     contours,_=cv2.findContours((gray<level).astype(np.uint8)*255,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
     choices=[]
     for c in contours:
         x,y,w,h=cv2.boundingRect(c);area=cv2.contourArea(c)
         if (not 2500<area<18000 or not .65<w/h<1.3 or min(x,y)<10
                 or x+w>638 or y+h>398 or np.linalg.norm(np.array([x+w/2,y+h/2])-expected)>radius):continue
         roi=gray[max(0,y-4):min(400,y+h+4),max(0,x-4):min(640,x+w+4)]
         rings=cv2.HoughCircles(cv2.GaussianBlur(roi,(5,5),1),cv2.HOUGH_GRADIENT,1,20,
             param1=40,param2=15,minRadius=int(w*.22),maxRadius=int(w*.38))
         if rings is None:continue
         for u,v,r in rings[0]:
             u+=max(0,x-4);v+=max(0,y-4)
             if abs(u-(x+w/2))>w*.12 or not y+h*.25<v<y+h*.55 or np.linalg.norm(np.array([u,v])-expected)>radius:continue
             choices.append(dict(center_px=[float(u),float(v)],bbox=[x,y,w,h],rim_radius_px=float(r),method='near_dark_body_inner_rim',trained_model_used=False))
     if len(choices)==1:return choices[0]
    raise ValueError('No unique near-view nut rim')


def locate_clipped_body(gray, expected, radius=65.):
    """Coarse centering ONLY; a clipped silhouette cannot authorize grasping."""
    choices=[]
    contours,_=cv2.findContours((gray<55).astype(np.uint8)*255,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        x,y,w,h=cv2.boundingRect(c);area=cv2.contourArea(c)
        center=np.array([x+w/2.,y+h/2.])
        if (2500<area<18000 and 60<w<150 and 50<h<160 and y+h>=398
            and x>10 and x+w<630 and np.linalg.norm(center-expected)<radius):
            choices.append(dict(center_px=center.tolist(),bbox=[x,y,w,h],coarse_only=True))
    if len(choices)!=1:raise ValueError('No unique clipped nut silhouette')
    return choices[0]


def track_consensus(frames,template,offset,predicted):
    """Require two consistent views; a single IR/specular flash is an outlier."""
    observations=[]
    for index,gray in enumerate(frames):
        try:
            value=track_template(gray,template,offset,minimum_score=.78,search_center=predicted,search_radius=25.)
            observations.append(dict(value,frame_index=index))
        except ValueError:pass
    if len(observations)<2 or max(v['score'] for v in observations)<.85:
        raise ValueError('Insufficient consistent template views')
    points=np.array([v['center_px'] for v in observations]);center=np.median(points,axis=0)
    if np.max(np.linalg.norm(points-center,axis=1))>1.5:raise ValueError('Nut image unstable')
    return observations
