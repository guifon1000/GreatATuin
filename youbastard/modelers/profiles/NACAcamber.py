import matplotlib.pyplot as plt
import numpy as np

N=15
tc=0.12
corde=1.0
m=0.00
p=0


x0=np.linspace(-1,1,N)
x1=[]
for i in range(0,N):
    x1.append((0.5*(-np.cos(i*np.pi/(N+1))+1.0)))




y1=(1/0.2)*tc*(0.2969*np.power(x1,0.5)-0.126*np.power(x1,1)-0.3516*np.power(x1,2)+0.2843*np.power(x1,3)-0.1015*np.power(x1,4))



plt.plot(x1,y1)
plt.axis('equal')
plt.show()
plt.clf()



#m=0.04
#p=0.4
yc=[]
teta=[]
for x in x1:
    x2=x*corde
    if x<=p and p!=0:
        yc.append(m*x*(2*p-x)/(p*p))
        teta.append(np.arctan(2*m*(p-x)/(p*p)))
    else:
        yc.append(m*(1-2*p+2*p*x-x*x)/((1-p)*(1-p)))
        teta.append(np.arctan(2*m*(p-x)/((1-p)*(1-p))))


xU=[]
yU=[]
xL=[]
yL=[]

for t in range(len(teta)):
    xU.append(corde*(x1[t]-y1[t]*np.sin(teta[t])))
    yU.append(corde*(yc[t]+y1[t]*np.cos(teta[t])))
    xL.append(corde*(x1[t]+y1[t]*np.sin(teta[t])))
    yL.append(corde*(yc[t]-y1[t]*np.cos(teta[t])))

#plt.plot(x1*corde,yc)

#plt.plot(x1*corde,yc+y1)
#plt.plot(x1*corde,-y1)
#plt.plot(x1*corde,y1)
plt.plot(xU,yU,'.r')
plt.plot(xL,yL,'.g')

plt.axis('equal')
print corde*x1[np.argmax(y1*corde)]
#plt.scatter(corde*x1[np.argmax(y1*corde)],0.5*(yU[np.argmax(y1*corde)]+yL[np.argmax(y1*corde)]))
centerPale=[corde*x1[np.argmax(y1*corde)],0.5*(yU[np.argmax(y1*corde)]+yL[np.argmax(y1*corde)])]

ep=[]
for i in range(len(yU)):
    ep.append(yU[i]-yL[i])

extrados=[xU[np.argmax(ep)],yU[np.argmax(ep)]]
ex2=[0.0,0.70]
plt.scatter(extrados[0],extrados[1])
plt.scatter(ex2[0],ex2[1])
vec=[ex2[0]-extrados[0],ex2[1]-extrados[1]]

plt.show()
st='NACAcamber'+str(int(m*100))+str(int(p*10))+str(int(tc*100))+'.dat'
f=open(st,'w')
f.write('profil'+'\n')
for i in range(1,len(x1)):
    f.write(str(xU[-i]+0.*vec[0])+' '+str(yU[-i]+0.*vec[1])+'\n')
for i in range(len(x1)):
    f.write(str(xL[i]+0.*vec[0])+' '+str(yL[i]+0.*vec[1])+'\n')
f.close()
