#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from Tkinter import *
import Tkinter as tk
#from TkTreectrl import * #To use this terminal:  apt-get install tktreectrl
import tkMessageBox as msg
import tf.transformations
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped
import os

def MsgBox(title, text, style):
    box = [
        msg.showinfo,       msg.showwarning,    msg.showerror,
        msg.askquestion,    msg.askyesno,       msg.askokcancel,        msg.askretrycancel,
    ];
    tk.Tk().withdraw(); #Hide Main Window.
    if style in range(7):
        return box[style](title, text)


def AddOnClick(event):
    global locationName
    global latestPose
    global latestOrientation
    global nameArray
    global poseArray
    global quaternion

    name= locationName.get()
    pose= latestPose
    orientation= latestOrientation
    quaternion=latestQuaternion
    if(name!=''and pose!=(0,0,0)and quaternion!=(0,0,0,0)):
        locationName.labelText =""
        locationName.delete(0, 'end')
        nameArray.append(name)
        poseArray.append(pose)
        orientationArray.append(orientation)
        quaternionArray.append(quaternion)
        warnLabel.config(text="")
        nameList.insert(nameList.size() ,"name: " +name+"   "+" pose: (" +str(pose.x)+","+str(pose.y)+","+str(pose.z)+")   "+" quaternion: " +str(quaternion))
        
    elif (pose==(0,0,0) or quaternion==(0,0,0,0)):
        #print "Please tag a point on the map first."
        warnLabel.config(text="Please tag a point on the map.")
    elif name=="":
        #print "Please type location name"
        warnLabel.config(text="Please type location name.")

def deleteOnClick(event):
    global nameList
    items = nameList.curselection()
    pos = 0
    for i in items :
        idx = int(i) - pos
        nameList.delete( idx,idx )
        pos = pos + 1
        for j in range(idx,(nameArray.__len__()-1)):
            nameArray[j]=nameArray[j+1]
            poseArray[j]=poseArray[j+1]
            orientationArray[j]=orientationArray[j+1]
            quaternionArray[j]=quaternionArray[j+1]
        nameArray[nameArray.__len__()-1]=None
        poseArray[nameArray.__len__()-1]=None
        orientationArray[nameArray.__len__()-1]=None
        quaternionArray[nameArray.__len__()-1]=None
        print "delete "+str(idx)
    print nameArray
def openFile(event):
    f = open('locationPoint', 'w')
    for line in f:
        print line

def saveOnClick(event):
    # pubDataStaus("updating")
    #print __file__
    dir = os.path.dirname(__file__)
    #print dir
    filename = dir+'/locationPoint.txt'
    #print filename
    ##for checking
    # if os.path.isfile(filename):
    #     print "locationPoint.txt "+"exists"
    # else:
    #     print "locationPoint.txtv "+"not exist"

    f = open(filename,'w')
    # else:
    #     try:
    #         f = open('locationPoint.txt','r+')   # Trying to create a new file or open one
    #     except:
    #         print('new text file error')
    f.truncate()
    for j in range(0, nameArray.__len__()):
        if nameArray[j] is not None:
            s = (nameArray[j]+ ","
            +str(poseArray[j].x)+","
            +str(poseArray[j].y)+","
            +str(poseArray[j].z)+","
            +str(quaternionArray[j].x)+","
            +str(quaternionArray[j].y)+","
            +str(quaternionArray[j].z)+","
            +str(quaternionArray[j].w)+"\n")
            #print s
            f.writelines(s)
    #print "done"
    global filePath
    filePath.config(text="File path: "+filename) 
    f.close()
    pubDataStaus(os.path.dirname(__file__)+'/locationPoint.txt')

def loadOnClick(event):
    pass

def getPoseData(data):
    print "getten"
    global latestPose
    latestPose = data.pose.pose.position
    global latestQuaternion
    latestQuaternion = data.pose.pose.orientation
    #print("Point Position: [ %f, %f, %f ]"%(latestPose.x, latestPose.y, latestPose.z))
    #print("Quat Orientation: [ %f, %f, %f, %f]"%(latestQuaternion.x, latestQuaternion.y, latestQuaternion.z, latestQuaternion.w))
    global latestOrientation
    latestOrientation = tf.transformations.euler_from_quaternion([latestQuaternion.x, latestQuaternion.y, latestQuaternion.z, latestQuaternion.w])
    #print("Euler Angles: %s"%str(latestOrientation))
    global poseLabel
    global nameList
    saveString = ("latest goal position: "+str(latestPose)+"\n Orientation: "+str(latestOrientation)+"\n")
    poseLabel.config(text=saveString)
    
def subscribePose():
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, getPoseData)
    # rospy.Subscriber('/move_base_simple/goal', PoseStamped, getPoseData)
    global background

def pubDataStaus(dataStatus): 
    pub = rospy.Publisher('WPsOK', String, queue_size=10)
    pub.publish(dataStatus)

def subscribePoint():
    rospy.Subscriber('/clicked_point', PointStamped, printclickpoint)

def printclickpoint(data):
    print "clicked_point"
    print data.point

def on_closing():
    if msg.askokcancel("Warning", "Do you want to quit?"):
        background.destroy()
        rospy.signal_shutdown("")

nameArray=[]
poseArray=[]
orientationArray=[]
quaternionArray=[]
latestPose=(0,0,0)
latestOrientation=(0,0,0)
latestQuaternion=(0,0,0,0)

background = tk.Tk()

#tk.iconbitmap(r'c:\Python32\DLLs\py.ico') #set icon
background.wm_title("Location Config")
background.geometry('{}x{}'.format(900,800))# window size
background.resizable(width=True, height=True) #window resizable


background.protocol("WM_DELETE_WINDOW", on_closing)

F1=Frame(background, height=60,width=600)
F1.pack()
titleLabel = Label(F1,text="Hints: 1. Press '2D Nav Goal' in RVIZ.\n2. Click the location you want to tag.\n3. Type the name of location below and press 'ADD'  \n",justify=LEFT,relief=RIDGE,width=80)
titleLabel.pack()

F2=Frame(background, height=20,width=900)
F2.pack()
poseLabel =Label(F2,text="Click the location you want to tag in RVIZ")
poseLabel.pack()

F3=Frame(background, height=20,width=900)
F3.pack()
nameLabel = Label(F3, text="Location Name")
nameLabel.grid(row=0, column=0)
locationName = Entry(F3, bd =5)
locationName.grid(row=0, column=1)
okBtn = Button(F3, text="ADD")
okBtn.grid(row=0, column=2)
okBtn.bind('<Button-1>', AddOnClick)
warnLabel = Label(F3,text='',fg = "red",bg = "yellow")
warnLabel.grid(row=0, column=3)


F4=Frame(background, height=200,width=600)
F4.pack()
scrollbar = Scrollbar(F4)
scrollbar.pack( side = RIGHT, fill=Y )
nameList =Listbox(F4,relief=RIDGE,width=600,height=30,yscrollcommand = scrollbar.set )

# for line in range(5):
#    nameList.insert(END, "Location " + str(line))

nameList.pack( side = LEFT, fill = BOTH )
scrollbar.config( command = nameList.yview )

F5=Frame(background, height=200,width=600)
F5.pack()
deleteBtn = Button(F5, text="DELETE")
deleteBtn.grid(row=0, column=0)
deleteBtn.bind('<Button-1>', deleteOnClick)
filePath = Label(F5,justify=LEFT,width=80,text="FILE PATH:")
filePath.grid(row=0, column=1)
saveBtn = Button(F5, text="SAVE")
saveBtn.grid(row=0, column=2)
saveBtn.bind('<Button-1>', saveOnClick)

saveBtn = Button(F5, text="LOAD")
saveBtn.grid(row=0, column=3)
saveBtn.bind('<Button-1>', loadOnClick)

if __name__ == '__main__':
    rospy.init_node('gui', anonymous=False)
    rospy.on_shutdown(background.quit)
    subscribePose()
    subscribePoint()
    background.mainloop()
    rospy.spin()
