import cv2
import numpy as np

color_gray=(120,120,120)
color_black=(0,0,0)
color_red=(0,0,255)
color_light_gray=(200,200,200)
font =cv2.FONT_HERSHEY_DUPLEX


map_empty=np.ones([800,500,3],np.uint8)*255
cv2.rectangle(map_empty, (220,0),(250,80),color_gray, -1)
cv2.rectangle(map_empty, (150,180),(270,210),color_gray, -1)
cv2.rectangle(map_empty, (370,120),(400,200),color_gray, -1)
cv2.rectangle(map_empty, (0,310),(200,340),color_gray, -1)

cv2.rectangle(map_empty, (300,460),(500,490),color_gray, -1)
cv2.rectangle(map_empty, (230,590),(350,620),color_gray, -1)
cv2.rectangle(map_empty, (100,600),(130,680),color_gray, -1)
cv2.rectangle(map_empty, (250,720),(280,800),color_gray, -1)

map_pt=map_empty.copy()

pt_self=[(70,60),  #0
(80,130),          #1
(170,130),         #2
(320,130),         #3
(320,60),          #4
(450,60),          #5
(450,260),         #6
(315,260),         #7
(210,260),         #8
(80,260),          #9
(440,330),         #10
(320,330),         #11
(450,160),         #12
(380,400),         #13
(290,400),         #14
(320,210),         #15
(170,60)]          #16
pt_enemy=[]
for i in range(len(pt_self)):
    x=500-pt_self[i][0]
    y=800-pt_self[i][1]
    pt_enemy+=[(x,y)]

pt_list=pt_self+pt_enemy+[(250,400)]
nei_matrix=np.ones([len(pt_list),len(pt_list)])*10000

def add_edge(i,j):
    map_draw_temp=map_pt.copy()
    
    cv2.line(map_draw_temp,pt_list[i],pt_list[j],color_light_gray,66)
    cv2.line(map_pt,pt_list[i],pt_list[j],color_red,2)
    distance=pow(pow((pt_list[i][0]-pt_list[j][0]),2)+pow((pt_list[i][1]-pt_list[j][1]),2),0.5)
    line_center=((pt_list[i][0]+pt_list[j][0])//2,(pt_list[i][1]+pt_list[j][1])//2)
    cv2.putText(map_pt,'%d'%(int(distance)),line_center,font,0.6,color_red)
    nei_matrix[i][j]=round(distance+0.1)
    nei_matrix[j][i]=round(distance+0.1)

    if (i<(len(pt_list)-1)/2) &( j<(len(pt_list)-1)/2):
        if (i != len(pt_list)-1):
            i=len(pt_self)+i
        if (j != len(pt_list)-1):    
            j=len(pt_self)+j

        #cv2.line(map_draw_temp,pt_list[i],pt_list[j],color_light_gray,66)
        cv2.line(map_pt,pt_list[i],pt_list[j],color_red,2)
        distance=pow(pow((pt_list[i][0]-pt_list[j][0]),2)+pow((pt_list[i][1]-pt_list[j][1]),2),0.5)
        line_center=((pt_list[i][0]+pt_list[j][0])//2,(pt_list[i][1]+pt_list[j][1])//2)
        # cv2.putText(map_pt,'%d'%(int(distance)),line_center,font,0.6,color_red)
        nei_matrix[i][j]=round(distance+0.1)
        nei_matrix[j][i]=round(distance+0.1)

    cv2.addWeighted(map_draw_temp,0.3,map_pt,0.7,0,map_pt)


def add_edges(source,*dsts):
    for dst in dsts:
        if dst>source:
            add_edge(source,dst)

add_edges(0,1,2,16,9)
add_edges(1,2,16,9,3)
add_edges(2,3,16)
add_edges(3,4,15,7,11)
add_edges(4,5,15,7,11)
add_edges(5,12,6,10)
add_edges(6,12,7,8,9,10,13,11,14,31,34,28,27)
add_edges(7,8,9,15,11,14,13,10,31,34,28)
add_edges(8,9,10,11,13)
add_edges(10,13,11,14,34,31,28,12,27,23)
add_edges(11,13,14,34,31,28,24,23,27)
add_edges(12,13)
add_edges(13,14,34,31,30,15) 
add_edges(14,15,34,31,30,28,27,24,23)
add_edges(15,34,28,24,32)


for i in range(len(pt_list)):
    cv2.circle(map_pt,pt_list[i],5,color_black,3)
    cv2.putText(map_pt,'%d'%i,pt_list[i],font,0.8,(255,0,0))


cv2.imwrite("map.jpg",map_pt)

pt_yaw = [0,  # 0
            0,  # 1
            1.57,  # 2
            0,  # 3
            0,  # 4
            0,  # 5
            -0.78,  # 6
            0,  # 7
            0,  # 8
            1.57,  # 9
            -1.0,  # 10
           999,  # 11
           0,  # 12
           -1.57,  # 13
           999,  # 14
           0,  # 15
           0]  # 16

point_matrix=np.zeros([len(pt_list),2])
point_yaw = np.zeros(len(pt_list))
for i in range(len(pt_list)):
    point_matrix[i][0] = pt_list[i][0]
    point_matrix[i][1] = pt_list[i][1]
    if i < int(len(pt_list)/2):
        point_yaw[i]=pt_yaw[i]
    else:
        if point_yaw[i-17]<0:
            point_yaw[i]=3.14+point_yaw[i-17]
        elif point_yaw[i-17]==999:
            point_yaw[i]=999
        else:
            point_yaw[i]= point_yaw[i-17]-3.14

point_yaw[len(pt_list)-1]=999

fs = cv2.FileStorage("matrix.xml", cv2.FILE_STORAGE_WRITE)
fs.write('Matrix',nei_matrix)
fs.write('Point',point_matrix)
fs.write('Point_yaw', point_yaw)
cv2.imshow("map",map_pt)
cv2.waitKey(0)
