import numpy as np
import pandas as pd
import cvxpy as cp
import os

# --------------------------
# Data Load
# --------------------------

# ---------------------------
# PARAMETERS (edit these to change vehicle/run)
vehicle_mass = 4.3           # kg (editable)
wheel_base = 0.33            # meter
mu = 0.9                     # friction coefficient (editable)
g = 9.81
v_init = 0.0          # m/s initial linear speed at theta0 (editable)
v_max = 15.0
a_max = 4.0                  # initial barrier weight
a_min = -4.0
wheel_positions_body = np.array([  # FL, FR, RL, RR positions [x_forward, y_right]
    [ 0.18,  0.15],
    [ 0.18, -0.15],
    [-0.18,  0.15],
    [-0.18, -0.15]
])
# ---------------------------
# ------- User Inputs -------
script_dir = os.path.dirname(os.path.abspath(__file__))
map_dir = script_dir + "/../../peripheral/racetracks/"
map_name = "Shanghai/Shanghai_"

csv_path = os.path.join(map_dir, map_name + "frenet_track.csv") # centerline
out_path = os.path.join(map_dir, map_name + "cvxpy_speedopted.csv") # centerline

def load_path(csv_path="path_xy.csv"):
    if os.path.exists(csv_path):
        df = pd.read_csv(csv_path)
        if {"x","y"}.issubset(df.columns):
            return df["x"].to_numpy(float), df["y"].to_numpy(float)
        else:
            arr = df.to_numpy(float)
            return arr[:,0], arr[:,1]
    else:
        raise FileNotFoundError(f"{csv_path} not found")

# --------------------------
# Uniform θ-grid
# --------------------------
def make_uniform_theta(n):
    theta = np.linspace(0,1,n)
    dtheta = theta[1]-theta[0]
    return theta,dtheta

# --------------------------
# Derivatives (#38,#39)
# --------------------------
def derivatives(x,y,dtheta):
    n=len(x)
    # midpoints count = n-1
    xdot_mid = (x[1:]-x[:-1])/dtheta
    ydot_mid = (y[1:]-y[:-1])/dtheta
    xdd_mid  = np.zeros(n-1)
    ydd_mid  = np.zeros(n-1)
    # #39 near left boundary
    xdd_mid[1]=(0.5*x[0]-x[1]+0.5*x[2])/(dtheta**2)
    ydd_mid[1]=(0.5*y[0]-y[1]+0.5*y[2])/(dtheta**2)
    # #38 6-point stencil interior
    for i in range(2,n-2):
        coeff=[-5/48,13/16,-17/24,-17/24,13/16,-5/48]
        ids=[i-3,i-2,i-1,i,i+1,i+2]
        xdd_mid[i]=sum(c*x[j] for c,j in zip(coeff,ids))/(dtheta**2)
        ydd_mid[i]=sum(c*y[j] for c,j in zip(coeff,ids))/(dtheta**2)
    # #39 near right boundary
    i=n-2
    xdd_mid[i]=(0.5*x[n-3]-x[n-2]+0.5*x[n-1])/(dtheta**2)
    ydd_mid[i]=(0.5*y[n-3]-y[n-2]+0.5*y[n-1])/(dtheta**2)
    return xdot_mid,ydot_mid,xdd_mid,ydd_mid

# --------------------------
# Curvature at midpoints
# --------------------------
def curvature_mid(xdot,ydot,xdd,ydd):
    num=xdot*ydd-ydot*xdd
    den=(xdot**2+ydot**2)**1.5
    kappa=np.zeros_like(num)
    m=den>1e-12
    kappa[m]=num[m]/den[m]
    return kappa

# --------------------------
# Initial condition (#32)
# --------------------------
def b0_from_eq32(v_init,x0,y0,x1,y1,dtheta):
    ds01=np.hypot(x1-x0,y1-y0)
    return ((v_init*dtheta)/max(ds01,1e-9))**2

def bn_from_eq32(v_final,xnm2,ynm2,xnm1,ynm1,dtheta):
    ds=np.hypot(xnm1-xnm2,ynm1-ynm2)
    return ((v_final*dtheta)/max(ds,1e-9))**2

# --------------------------
# Solver (CVXPY)
# --------------------------
def solve_speed_profile(x,y,mu=0.9,g=9.81,vmax=6.0,
                        a_max=4.0,a_min=-6.0,v_init=0.0):
    n=len(x)
    theta,dtheta=make_uniform_theta(n)
    xdot_mid,ydot_mid,xdd_mid,ydd_mid=derivatives(x,y,dtheta)
    kappa_mid=curvature_mid(xdot_mid,ydot_mid,xdd_mid,ydd_mid)

    # CVX vars
    b=cp.Variable(n,pos=True)

    # #29 objective
    # obj=cp.sum(2*dtheta/(cp.sqrt(b[:-1])+cp.sqrt(b[1:])))
    obj = cp.sum( dtheta * cp.power(b[:-1], -0.5) )


    constraints=[]
    # #32 init & final
    b0=b0_from_eq32(v_init,x[0],y[0],x[1],y[1],dtheta)
    # bn=bn_from_eq32(v_final,x[-2],y[-2],x[-1],y[-1],dtheta)
    constraints+=[b[0]==b0] # ,b[-1]==bn

    # #38 accel constraint
    for i in range(1,n):
        ai=(b[i]-b[i-1])/(2*dtheta)
        constraints+=[ai<=a_max,ai>=a_min]

    # #39 nodewise bounds from curvature
    vcap_sq=np.full(n-1,vmax**2)
    m=np.abs(kappa_mid)>1e-9
    vcap_sq[m]=np.minimum(vcap_sq[m],(mu*g)/np.abs(kappa_mid[m]))
    b_max_node=np.full(n,vmax**2)
    b_max_node[0]=min(vmax**2,vcap_sq[0])
    b_max_node[-1]=min(vmax**2,vcap_sq[-1])
    for i in range(1,n-1):
        b_max_node[i]=min(vmax**2,vcap_sq[i-1],vcap_sq[i])
    for i in range(n):
        constraints+=[b[i]<=b_max_node[i]+1e-9,b[i]>=1e-9]

    prob=cp.Problem(cp.Minimize(obj),constraints)
    try:
        prob.solve(solver=cp.ECOS,abstol=1e-7,reltol=1e-7,feastol=1e-7,max_iters=2000,verbose=True)
    except Exception:
        prob.solve(solver=cp.SCS,verbose=True)

    if b.value is None:
        raise RuntimeError("Solver failed.")
    b_sol=np.maximum(b.value,0)
    # speed = ||s'|| * sqrt(b) at nodes
    spd_mid=np.hypot(xdot_mid,ydot_mid)
    spd_node=np.zeros(n)
    spd_node[0]=spd_mid[0]; spd_node[-1]=spd_mid[-1]
    for i in range(1,n-1):
        spd_node[i]=0.5*(spd_mid[i-1]+spd_mid[i])
    v_node=spd_node*np.sqrt(b_sol)
    return v_node,b_sol,kappa_mid

# --------------------------
# Main: load, solve, save
# --------------------------
def main():
    x,y=load_path(csv_path)
    v,b,kappa=solve_speed_profile(x,y,a_max=a_max, a_min=a_min, v_init=v_init, vmax=v_max)

    # kappa 길이 보정
    if len(kappa)==len(x)-1:
        kappa_nodes=np.zeros(len(x))
        kappa_nodes[0]=kappa[0]
        kappa_nodes[1:-1]=kappa
        kappa_nodes[-1]=kappa[-1]
    else:
        kappa_nodes=kappa

    df=pd.DataFrame({"x":x,"y":y,"v":v, "kappa": kappa_nodes})
    df.round({
        'x':3,
        'y':3,
        'kappa':4,
        'v':3
    }).to_csv(out_path,index=False)
    print("Saved path with velocity profile: ", out_path)

if __name__=="__main__":
    main()
