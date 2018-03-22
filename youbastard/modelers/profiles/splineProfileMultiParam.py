import numpy as np
from scipy.special import binom
from scipy.interpolate import interp1d
import sys
import matplotlib.pyplot as plt
from youbastard.geometry.Polyline2D import Polyline2D
name="foil"
par1=0.1
par2=0.05
par3=0.01
par4=0.001




def Bernstein(n,k):
    coeff=binom(n,k)
    def _bpoly(x):
        return coeff*x**k*(1-x)**(n-k)
    return _bpoly


def Bezier(points, num=100):
    N=len(points)
    t=np.linspace(0,1,num=num)
    curve=np.zeros((num,2))
    for ii in range(N):
        curve+=np.outer(Bernstein(N-1,ii)(t),points[ii])
    return curve


class Profile(object):
    """
    an object returning intrados and extrados, from leading edge to trailing edge, and the chord is 1
    """
    def __init__(self , **kwargs ):
        if kwargs.has_key('npt'):
            npt=int(kwargs['npt'])
        else:
            npt=10
        if kwargs.has_key('typ'):
            if kwargs['typ'] =='fon':
                if kwargs.has_key('par'):
                    self.x,self.extra,self.intra = fonProfile(kwargs['par'],npt)
            elif kwargs['typ'] == 'naca4d' :
                print 'naca 4 digits'
                if kwargs.has_key('par'):
                    digits = [int(s) for s in list(kwargs['par'])]
                    self.x,self.extra,self.intra = Naca4DigitsProfile(digits,npt)
                else : 
                    print '\n\n\nyou asked me for a naca 4 digits profile without giving me the parameters, \n',\
                            '...\n\n'

    def polyline(self, closed = True):
        l = []
        revX = self.x[::-1]
        revextra = self.extra[::-1]
        for i in range(len(revX)):
            p = (revX[i], revextra[i])
            l.append(p)
        for i in range(1,len(self.x)):
            p = (self.x[i], self.intra[i])
            l.append(p)
        pl = Polyline2D(l,closed = closed)
        return pl

def Naca4DigitsProfile(digits,N):
    corde = 1.0
    x1=[]
    for i in range(N):
        #x1.append(float(i)/(N-1))
        x1.append((0.5*(-np.cos(i*np.pi/(N-1))+1.0)))
    strtc = str(digits[-2])+str(digits[-1])
    tc = 0.01*float(strtc)
    y1=(1/0.2)*tc*(0.2969*np.power(x1,0.5)\
       -0.126*np.power(x1,1)\
       -0.3516*np.power(x1,2)\
       +0.2843*np.power(x1,3)\
       -0.1015*np.power(x1,4))



    u,v=Bezier(list(zip(x1,y1)),N).T


    m=0.01*float(digits[0])
    p=0.1*float(digits[1])
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
    return xL,yL,yU
    #plt.plot(x1,y1,'k.')
    #plt.plot(x1,-y1,'k.')
    #plt.plot(xU,yU,'r.')
    #plt.plot(xL,yL,'r.')
    #plt.axis('equal')
    #plt.show() 





def fonProfile(par,N,**kwargs):
    if len(par)%2==0:
        print 'the number of parameters must be odd, sucker'
    pe = []
    pc = []
    alphaLE=float(par[0])
    for v in par[1:len(par)/2+1]:pe.append(float(v))
    for v in par[len(par)/2+1:len(par)+1]:pc.append(float(v))
    N0=len(pe)
    N1=len(pc)
    x0=np.linspace(1./(N0+1.),1.,num=N0,endpoint=False )   #parametres epaisseur
    x1=np.linspace(1./(N1+1.),1.,num=N1,endpoint=False )   #parametres corde

    
    xe=[]
    ye=[]
    xe.append(0.)
    ye.append(0.)
    xe.append(0.000)
    ye.append(alphaLE*(0.*np.min(pe)+np.max(pe)))
    for i in range(len(x0)):
        xe.append(x0[i])
        ye.append(pe[i])

    epTE=0.005

    xe.append(1.)
    ye.append(epTE)


    xc=[]
    yc=[]

    xc.append(0.)
    yc.append(0.)
    for i in range(len(x1)):
        xc.append(x1[i])
        yc.append(pc[i])
    xc.append(1.)
    yc.append(0.)


    f=interp1d(xc,yc,kind='cubic')
    #f._call_spline(
    x2see = f.x
    y2see = f.y
    #plt.clf()
    #plt.plot(x2see,y2see)
    u,v=Bezier(list(zip(xe,ye)),N).T
    intrados=f(u)-0.5*v
    extrados=f(u)+0.5*v
    #plt.plot(u[::-1],extrados)
    #plt.plot(u,intrados)
    #plt.axis('equal')
    #plt.show()
    return u,extrados,intrados 
        
        
if __name__=='__main__':
    profils = []
    f = Profile(typ = 'naca4d',par = '4416',npt = 50) 
    profils.append(f)
    f = Profile(typ = 'fon',par = [0.82,0.21,0.13,0.04,0.029],npt = 50) 
    profils.append(f)
    cols =['k*-','r*-']
    for i,p in enumerate(profils):
        plt.plot(p.x,p.extra,cols[i])
        plt.plot(p.x,p.intra,cols[i])
    plt.axis('equal')
    plt.show()

    

