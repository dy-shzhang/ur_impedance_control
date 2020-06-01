import socket
import time
HOST ='192.168.1.50'
PORT =30003
ADDR=(HOST,PORT)
robot = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
robot.connect(ADDR)



def ur_movel(p,a=0.12,v=0.02,t=0,r=0):
    str_command = "movel(p[%(x)s,%(y)s,%(z)s,%(rx)s,%(ry)s,%(rz)s],a=%(a)s,v=%(v)s,t=%(t)s,r=%(r)s) \n" %{'x':p[0],'y':p[1],'z':p[2],'rx':p[3],'ry':p[4],'rz':p[5],'a':a,'v':v,'t':t,'r':r}
    robot.send(str_command.encode())

p=[-0.11865,-0.46516,0.30444,0.0721,-3.1324,0.0551]

count =0
print(p)
while(True):
    count+=1
    ur_movel(p)
    print("order send")
    time.sleep(1)
    p2 = [0.11865,-0.46516,0.30444,0.0721,-3.1324,0.0551]
    ur_movel(p2)
    print("secondorder send")
    time.sleep(1)
    if(count==5):
        print("to stop")
        str_command ="speedj([0.,0.,0.,0.,0.,0.],3,0)\n"
        robot.send(str_command.encode())
        break
