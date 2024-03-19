import math
from operator import itemgetter
from photonlibpy.photonCamera import PhotonCamera

from wpimath.geometry import Transform3d, Rotation3d
from wpimath import units
from commands2 import Subsystem, CommandScheduler
from typing import Tuple

from config import rear_camera_angle_offset, second_camera_height


class Vision(Subsystem):
    def __init__(self, scheduler: CommandScheduler, camera_name="Camera_Module_v1"):
        self.camera = PhotonCamera(camera_name)
    


    def get_distance_degree(self) -> Tuple[float, float] | None:
        result = self.camera.getLatestResult()
        # note = {"dist": value, "angle": value}
        all_distances = []
        if len(result.getTargets()) >= 1:
            for target in result.getTargets():
                target_to_cam_angle = target.getPitch()
                distance = second_camera_height/(math.tan(math.radians(target_to_cam_angle)))
                angle = target.getYaw()
                final_angle = angle + rear_camera_angle_offset
                all_distances.append({"dist": distance, "angle": final_angle})
        
            final_distances = sorted(all_distances, key=itemgetter('dist'))
            closest_note = final_distances[0]
            return closest_note
        
        else:
            return None
        
        
        

        
  
        


            
            


            



