"""One-height next-view exploration. Every target and stop reason is recorded.

Potential gain is a ranking heuristic, never evidence that unknown space is
free. Inspection views are counted only from recent measured obstacle hits.
"""
import math
from cf2_coverage import CoverageMission
from mission_manager.mission import NAVIGATE, LANDING, ABORTED

class ExplorationMission(CoverageMission):
    def __init__(self, origin, area, height, timeout=30, budget=180,
                 target_percent=80, no_progress_seconds=45, inspect=True, clearance=.25,
                 warning=.5, mapping_radius=.8):
        if not math.isfinite(target_percent) or not 1 <= target_percent <= 99 or not math.isfinite(no_progress_seconds) or no_progress_seconds <= 0:
            raise ValueError('Coverage target must be 1..99 percent; progress timeout must be positive')
        super().__init__(origin,area,height,timeout=timeout,budget=budget)
        self.targets,self.pending,self.deferred=[],[],[]
        self.phase='EXPLORE'; self.target_percent=target_percent
        self.no_progress_seconds,self.inspect=no_progress_seconds,inspect
        self.clearance,self.warning,self.radius=clearance,warning,mapping_radius
        self.tried={}; self.objects=[]; self.selected_object=None
        self.end_reason=''; self.progress_at=None; self.best_known=0
        self.last_refresh=None; self.cached_percent=0.; self.last_selection='EXPLORE'

    def start(self,now):
        self.started=self.goal_started=self.progress_at=now

    def _finish(self,now,reason):
        self.end_reason=reason
        self.active=None;self.phase='RETURN';self.goal_started=now

    def _key(self,p): return (round(p[0]/.2),round(p[1]/.2))

    def refresh(self,now,position,snapshot,force=False):
        if self.phase=='RETURN' or self.state!=NAVIGATE: return
        if not force and self.last_refresh is not None and now-self.last_refresh<.5: return
        self.last_refresh=now
        if snapshot['origin'] is None: return
        summary=super().summary(snapshot)
        self.cached_percent=summary['observed_percent']
        known=summary['observed_cells']
        if known>self.best_known:
            self.best_known,self.progress_at=known,now
        if now-self.progress_at>=self.no_progress_seconds:
            self._finish(now,'no_information_progress');return
        grid=snapshot['data'];w=snapshot['width'];res=snapshot['resolution'];ox,oy=snapshot['origin']
        def xy(i): return (ox+(i%w+.5)*res,oy+(i//w+.5)*res)
        def cell(p):
            x,y=math.floor((p[0]-ox)/res),math.floor((p[1]-oy)/res)
            return y*w+x if 0<=x<w and 0<=y<w else None
        def inside(p):
            return all(abs(v-c)<=length/2-.3 for v,c,length in zip(p,self.origin,self.area))
        survey_cells={i for i in range(len(grid))
                      if all(abs(v-c)<=length/2 for v,c,length in zip(xy(i),self.origin,self.area))}
        occupied={i for i,v in enumerate(grid) if v>=50}
        # Connected occupied components are observations, not object identities.
        components=[];remaining=set(occupied)
        while remaining:
            stack=[remaining.pop()];group=[]
            while stack:
                i=stack.pop();group.append(xy(i));x,y=i%w,i//w
                for dx in (-1,0,1):
                    for dy in (-1,0,1):
                        if 0<=x+dx<w and 0<=y+dy<w:
                            j=(y+dy)*w+x+dx
                            if j in remaining: remaining.remove(j);stack.append(j)
            components.append(group)
        current=[]
        for points in components:
            centre=tuple(sum(p[a] for p in points)/len(points) for a in (0,1))
            track=next((o for o in self.objects if math.dist(o['centre'],centre)<.3),None)
            if track is None:
                track={'id':len(self.objects),'centre':centre,'view_bins':[], 'attempted_bins':[],
                       'status':'partial'};self.objects.append(track)
            track['centre']=centre;track['last_seen_seconds']=now-self.started
            track['bounds']=[min(p[0] for p in points),min(p[1] for p in points),
                             max(p[0] for p in points),max(p[1] for p in points)]
            for observation in snapshot.get('recent_observations',[]):
                if math.dist(observation['endpoint'],centre)<.2:
                    observer=observation['observer']
                    sector=round(math.atan2(observer[1]-centre[1],observer[0]-centre[0])/(math.pi/4))%8
                    if sector not in track['view_bins']:
                        track['view_bins'].append(sector);self.progress_at=now
            current.append(track)
        # The active target stays fixed until reached, blocked or timed out.
        if self.active is not None: return
        def admissible(p,known_free=False):
            i=cell(p)
            if i is None or not inside(p) or self.tried.get(self._key(p),0)>=2:return False
            if known_free and grid[i]!=0:return False
            if grid[i]>=50:return False
            return all(math.dist(p,xy(j))>self.clearance+res for j in occupied)
        def gain(p):
            cx,cy=math.floor((p[0]-ox)/res),math.floor((p[1]-oy)/res)
            radius=math.ceil(self.radius/res)
            return sum(grid[y*w+x]<0 for y in range(max(0,cy-radius),min(w,cy+radius+1))
                for x in range(max(0,cx-radius),min(w,cx+radius+1))
                if y*w+x in survey_cells and (x-cx)**2+(y-cy)**2<=radius**2)
        inspections=[]
        if self.inspect and (self.last_selection!='INSPECT' or self.cached_percent>=self.target_percent):
            for o in current:
                if len(o['view_bins'])>=4:continue
                radius=max(self.warning+.1,.45)
                for sector in range(8):
                    if sector in o['view_bins'] or sector in o['attempted_bins']:continue
                    a=sector*math.pi/4
                    p=(o['centre'][0]+radius*math.cos(a),o['centre'][1]+radius*math.sin(a))
                    if .15<math.dist(position[:2],p)<=1.2 and admissible(p,True):
                        inspections.append((math.dist(position[:2],p),p,o,sector))
        if inspections:
            _,target,o,sector=min(inspections,key=lambda v:v[0])
            o['attempted_bins'].append(sector);self.selected_object=o['id'];self.phase='INSPECT'
        else:
            if self.cached_percent>=self.target_percent:
                self._finish(now,'coverage_target_reached');return
            candidates=[]
            # Select nearby known-free viewpoints beside unknown patches.
            stride=max(1,round(.15/res))
            for y in range(0,w,stride):
                for x in range(0,w,stride):
                    i=y*w+x;p=xy(i);d=math.dist(position[:2],p)
                    if grid[i]==0 and .2<=d<=1.2 and admissible(p,True):
                        g=gain(p)
                        if g>0:candidates.append((g/(.3+d),p))
            # If no known-free frontier is usable, probe a short unknown step.
            # A* and the reactive supervisor still control the actual movement.
            if not candidates:
                for sector in range(8):
                    a=sector*math.pi/4;p=(position[0]+.45*math.cos(a),position[1]+.45*math.sin(a))
                    if admissible(p):
                        g=gain(p)
                        if g>0:candidates.append((g,p))
            if not candidates:
                self.phase='EXPLORE';return
            _,target=max(candidates,key=lambda v:v[0]);self.phase='EXPLORE';self.selected_object=None
        self.active=len(self.targets);self.targets.append(target)
        self.tried[self._key(target)]=self.tried.get(self._key(target),0)+1
        self.goal_started=now;self.last_selection=self.phase

    def step(self,now,position,health,safety_state,planner_status):
        if safety_state in (3,4):self.state,self.reason=ABORTED,'safety_land';return
        if self.phase!='RETURN' and self.budget is not None and now-self.started>=self.budget:
            self._finish(now,'flight_budget_reached')
        goal=self.current_waypoint
        if goal is None:return
        if math.dist(position,(goal.x,goal.y,goal.z))<=.10:
            if self.phase=='RETURN':
                self.state,self.reason=LANDING,self.end_reason+'_returned'
            else:
                self.visited.append(self.active);self.active=None;self.index=len(self.visited)
        elif now-self.goal_started>self.timeout:self.skip(now,'progress_timeout')

    def skip(self,now,reason):
        if self.phase=='RETURN':self.state,self.reason=LANDING,'return_unresolved:'+reason;return
        if self.active is not None:
            self.unresolved.append({'target':self.targets[self.active],'reason':reason,
                                    'phase':self.phase,'object_id':self.selected_object})
        self.active=None;self.goal_started=now

    def prepare_repair(self,*args): pass

    def summary(self,snapshot):
        s=super().summary(snapshot)
        s.update(mode='explore',target_percent=self.target_percent,finish_trigger=self.end_reason,
                 no_progress_seconds=self.no_progress_seconds,obstacles=self.objects,
                 active_object_id=self.selected_object,inspection_complete=False)
        return s
