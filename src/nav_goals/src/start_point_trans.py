from math import asin,degrees
import rospy 

class start_point_trans:
    def __init__(self,axle_trans_angle):
        self.ata=axle_trans_angle
    
    def fromOriginalToMarked(self,origin_point,orientation):
        
        start_point_transed = 0 
        
        orient_angle =self.ata + degrees(2*asin(orientation))
       
        
        if origin_point==1:
            if abs(orient_angle)<=30:
                start_point_transed=1
            elif -120 <= orient_angle <= -60:
                start_point_transed=2
        
        elif origin_point==2:
            if abs(orient_angle)<=30:
                start_point_transed=3
            elif -108 <= orient_angle <= -48:
                start_point_transed=4
            elif abs(orient_angle)>=150:
                start_point_transed=5
        
        elif origin_point==3:
            if 29 <= orient_angle <= 80:
                start_point_transed=31
            elif -161 <= orient_angle <= -100:
                start_point_transed=32

        elif origin_point==4:
            if abs(orient_angle)>=150:
                start_point_transed=9
            elif -120 <= orient_angle <= -60:
                start_point_transed=10
        
        elif origin_point==5:
            if  60<= orient_angle <= 120:
                start_point_transed=11
            elif 124 <= orient_angle <= 179:
                start_point_transed=13
            elif -120 <= orient_angle <= -60:
                start_point_transed=12

        elif origin_point==6:
            if abs(orient_angle)>=150:
                start_point_transed=18
            elif 60 <= orient_angle <= 120:
                start_point_transed=17
        
        elif origin_point==7:
            if abs(orient_angle)<=30:
                start_point_transed=19
            elif 80 <= orient_angle <= 142:
                start_point_transed=20
            elif abs(orient_angle)>=150:
                start_point_transed=21

        elif origin_point==8:
            if abs(orient_angle)<=20:
                start_point_transed=22
            elif 29 <= orient_angle <= 80:
                start_point_transed=23
            elif abs(orient_angle)>=150:
                start_point_transed=24

        elif origin_point==9:
            if abs(orient_angle)<=30:
                start_point_transed=25
            elif 60 <= orient_angle <= 120:
                start_point_transed=26
        
        elif origin_point==10:
            if 80 <= orient_angle <= 142:
                start_point_transed=27
            elif 29 <= orient_angle <= 80:
                start_point_transed=28
            elif -108 <= orient_angle <= -48:
                start_point_transed=29
            elif -161 <= orient_angle <= -100:
                start_point_transed=30
            
            
        
        elif origin_point==11:
            if 29 <= orient_angle <= 80:
                start_point_transed=14
            elif -161 <= orient_angle <= -100:
                start_point_transed=16
            elif -56 <= orient_angle <= -1:
                start_point_transed=15

      

        
        if start_point_transed==0:
            print "////////////////ERROR! WRONG INPUT! RETRY ONCE/////////////////////////"
         

        #test=self.ata + origin_point + orientation
        return start_point_transed