import pcl
from datetime import datetime

cloud=pcl.load("./data/000001.pcd")

clipper=cloud.make_cropbox()
outcloud=pcl.PointCloud()



for line in open("./data/000001.txt"):
    #print(line)
    lable=line.split()

    type=lable[0]
    truncated=lable[1]
    occuluded=lable[2]
    alpha=float(lable[3])
    xmin=float(lable[4])
    ymin=float(lable[5])
    xmax=float(lable[6])
    ymax=float(lable[7])
    h=float(lable[8])
    w=float(lable[9])
    l=float(lable[10])
    x=float(lable[11])
    y=float(lable[12])
    z=float(lable[13])
    rotation_y=float(lable[14])

    if(type=="DontCare"):
        continue
    else:
        start=datetime.now()
        print(start)
        clipper.set_Rotation(0,rotation_y,0)
        clipper.set_MinMax(-l/2.0,-w/2.0,0,0,l/2.0,w/2.0,h,0)
        outcloud=clipper.filter()
        end=datetime.now()
        print(end)
        diff=(end-start)
        print(diff.microseconds/1000.0)
        print(outcloud.size)
    



