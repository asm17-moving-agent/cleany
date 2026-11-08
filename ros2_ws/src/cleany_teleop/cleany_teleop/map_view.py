"""Map geometry and occupied-cell checks shared by the phone display and goals."""
from dataclasses import dataclass
import hashlib
import math


@dataclass
class MapView:
    width: int
    height: int
    resolution: float
    x: float
    y: float
    yaw: float
    cells: list[int]

    def __post_init__(self):
        if (self.width <= 0 or self.height <= 0 or len(self.cells) != self.width*self.height
                or not all(math.isfinite(v) for v in (self.resolution,self.x,self.y,self.yaw))
                or self.resolution <= 0
                or any(type(v) is not int or not -1<=v<=100 for v in self.cells)):
            raise ValueError('Invalid map geometry')

    @property
    def revision(self):
        data = str((self.width,self.height,self.resolution,self.x,self.y,self.yaw)).encode()
        return hashlib.sha256(data+bytes(v+1 for v in self.cells)).hexdigest()[:24]

    def validate_goal(self, payload):
        if payload.get('map_id') != self.revision:
            raise ValueError('지도가 변경됐습니다. 목표를 다시 선택하세요.')
        values = [payload.get(k) for k in ('x','y','yaw')]
        if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
            raise ValueError('목표 좌표가 올바르지 않습니다.')
        dx,dy=values[0]-self.x,values[1]-self.y
        u=(math.cos(self.yaw)*dx+math.sin(self.yaw)*dy)/self.resolution
        v=(-math.sin(self.yaw)*dx+math.cos(self.yaw)*dy)/self.resolution
        if not (0<=u<self.width and 0<=v<self.height):
            raise ValueError('지도 밖의 목표입니다.')
        if self.cells[int(v)*self.width+int(u)] != 0:
            raise ValueError('장애물 또는 미확인 영역은 선택할 수 없습니다.')
        return tuple(float(v) for v in values)

    def snapshot(self):
        return dict(map_id=self.revision,width=self.width,height=self.height,
                    resolution=self.resolution,origin=[self.x,self.y,self.yaw],cells=self.cells)
