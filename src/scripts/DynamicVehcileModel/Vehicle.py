def UnitConversion():
     # mm per inch
    mm_pr_in = 25.4

    # m per inch
    m_pr_in = mm_pr_in/1000

    # N per lb
    lb_pr_N = 0.2248

    # lb per N
    N_pr_lb = 4.4482

    # m/s per km/h
    ms_pr_kmh = 1/3.6
    
    # m/s per mph
    ms_pr_mph = 0.44704
    
    # Nm per ftlbs
    Nm_pr_ftlbs = 1.356
    
    return mm_pr_in, m_pr_in, N_pr_lb, lb_pr_N, ms_pr_kmh, ms_pr_mph, Nm_pr_ftlbs


def VehicleData():
    import numpy as np
    
    mm_pr_in, m_pr_in, N_pr_lb, lb_pr_N, ms_pr_kmh, ms_pr_mph, Nm_pr_ftlbs = UnitConversion()
    
    # Wheelbase [m]
    L = 1.152

    # Curb weight [kg]
    M = 150

    # Gravitational acceleration [m/s^2]
    g = 9.81

    # Total weight [N]
    F_g = M*g

    # Yaw moment of inertia [kg*m^2]
    I_zz = M/4*L**2

    # Center of gravity height [m]
    CG_z = 0.3

    # Center of gravity longitudinally [m]
    CG_x = 0

    # Center of gravity transversely [m]
    CG_y = 0

    # a and b
    l_f = L/2 - CG_x
    l_r = L/2 + CG_x

    # Weight on the front wheels [N]
    N_fs = l_r*F_g/L

    # Weight on the rear wheels [N]
    N_rs = l_f*F_g/L

    # 1 wheel
    #F_yf1 = np.array([0,400])
    F_yf1 = np.array([0,801])
    #F_yf1 = np.array([0,100])
    alpha_f1 = np.array([0,-1])*np.pi/18

    # Cornering stiffness for 1 wheel [N/rad]
    C_alpha_f = -(F_yf1[0]-F_yf1[1])/(alpha_f1[0]-alpha_f1[1])
    C_alpha_r = -(F_yf1[0]-F_yf1[1])/(alpha_f1[0]-alpha_f1[1])

    # Understeer gradient [rad]
    K = N_fs/C_alpha_f - N_rs/C_alpha_r
    
    # Width between winches [m]
    w = 0.467
    
    # Wheel radius
    r_w = 0.0508
    
    return L, M, g, F_g, I_zz, CG_z, CG_x, CG_y, l_f, l_r, N_fs, N_rs, C_alpha_f, C_alpha_r, K, w, r_w


# Linear tire model
def LTire(alpha, mu, N, C_alpha):
    import numpy as np

    skid = 0

    F_y = -C_alpha*alpha

    # Test for skid
    if np.abs(F_y) > mu*N:
        skid = 1
        Fy = -np.sign(alpha)*mu*N

    return F_y, skid


# Nonlinear tire model
def NLTire(alpha, mu, N, C_alpha):
    import numpy as np
    
    skid = 0
    
    F_y = -C_alpha*alpha
    
    if np.abs(F_y) >= mu*N/2:
        if alpha == 0:
            F_y = 0
        else:
            F_y = -mu*N*np.sign(alpha)*(1 - mu*N/(4*C_alpha*np.abs(np.tan(float(alpha)))))
    
    # Test for skid
    if np.abs(F_y) > 0.9*mu*N:
        skid = 1
    
    return F_y, skid


def findTire(gamma, l_f, l_r, X, Y):
    import numpy as np
    
    # Coordinate of tire in initial coordinate system. Positive clockwise rotation including translation of CG
    X_f = X + l_f*np.cos(gamma)
    Y_f = Y + l_f*np.sin(gamma)
    
    X_r = X - l_r*np.cos(gamma)
    Y_r = Y - l_r*np.sin(gamma)

    return X_f, Y_f, X_r, Y_r


def vacuumForce(N_s, F, F_le, F_te, F_yf, F_xf, F_yr, F_g, gamma_le, gamma_te, delta_f, gamma, l_f, l_r, w):
    import numpy as np

    # Forces
    F_res_x = F_le*np.cos(gamma_le) + F_te*np.cos(gamma_te) + F_xf - F_g*np.cos(gamma)
    F_res_y = F_le*np.sin(gamma_le) - F_te*np.sin(gamma_te) + F_yf + F_yr + F_g*np.sin(gamma)
    F_res_plane = np.sqrt(F_res_x**2 + F_res_y**2)

    if F_res_plane<F:
        F_Vx = -F_res_x
        F_Vy = -F_res_y
    else:
        F_Vx = -F*F_res_x/F_res_plane
        F_Vy = -F*F_res_y/F_res_plane

    # Moments
    M_res = l_f*(F_yf + F_le*np.sin(gamma_le) - F_te*np.sin(gamma_te)) + l_r*F_yr + w/2*(F_te*np.cos(gamma_te) -F_le*np.cos(gamma_le))

    M_friction = 0.305*F

    if M_res<M_friction:
        M_V = -M_res
    else:
        M_V = -np.sign(M_res)*M_friction


    return F_Vx, F_Vy, M_V


def winchAngles(X, Y, gamma, w, l_f):
    import numpy as np

    # Coordinate of tire in initial coordinate system. Positive clockwise rotation including translation of CG
    xhat = l_f
    yhat = w/2
    X_le = X + xhat*np.cos(gamma) - yhat*np.sin(gamma)
    Y_le = Y + xhat*np.sin(gamma) + yhat*np.cos(gamma)
    le_vec = [X_le, Y_le]
    
    xhat = l_f
    yhat = -w/2
    X_te = X + xhat*np.cos(gamma) - yhat*np.sin(gamma)
    Y_te = Y + xhat*np.sin(gamma) + yhat*np.cos(gamma)
    te_vec = [X_te, Y_te]

    le_attach = [30, 2]
    te_attach = [30, -2]

    gamma_le = np.arctan2(le_attach[1]-le_vec[1],le_attach[0]-le_vec[0]) - gamma
    gamma_te = -(np.arctan2(te_attach[1]-te_vec[1],te_attach[0]-te_vec[0])) + gamma

    return gamma_le, gamma_te, X_le, Y_le, X_te, Y_te, le_attach, te_attach

def pid(prev_error_vel, error_vel, integral_vel, prev_error_yaw, error_yaw, integral_yaw):

    kp_vel = 10000
    ki_vel = 10
    kd_vel = 10
    
    kp_yaw = 15000
    ki_yaw = 5
    kd_yaw = 1

    diff_error_vel = (error_vel - prev_error_vel)
    diff_error_yaw = (error_yaw - prev_error_yaw)

    pid_vel = kp_vel*error_vel + kd_vel*diff_error_vel + ki_vel*sum(integral_vel)
    pid_yaw = kp_yaw*error_yaw + kd_yaw*diff_error_yaw + ki_yaw*sum(integral_yaw)

    F_le = pid_vel - pid_yaw
    F_te = pid_vel + pid_yaw

    return F_le, F_te

# Get data from a steer
def steerFunc(delta_f, vel_d, yaw_d, N_s, F, t_interval, t_span, tire_model='nonlinear', winch=False, mu=1, Yzero=[0,0,0,0,0,0]):
    from scipy.integrate import solve_ivp
    from scipy.interpolate import interp2d
    import numpy as np
    
    L, M, g, F_g, I_zz, CG_z, CG_x, CG_y, l_f, l_r, N_fs, N_rs, C_alpha_f, C_alpha_r, K, w, r_w = VehicleData()
    
    # Rear stearing
    delta_r = 0

    if not(winch):
        N_f = N_s/2
        N_r = N_s/2
    else:
        N_f = N_s/2
        N_r = N_s/2
        dt = t_span[1] - t_span[0]
        integral_vel = []
        vel = [Yzero[0]]
        integral_yaw = []
        yaw = [Yzero[3]]
    

    # Define function to take the ODE system and time interval
    def dYdt(t, Y):
        y1,y2,y3,y4,y5,y6 = Y

        # Slip angles
        alpha_f = (y2 + l_f*y3)/y1  - delta_f(t)
        alpha_r = (y2 - l_r*y3)/y1
        
        # Tire forces
        if tire_model == 'linear':
            F_tyf, skid_f = LTire(alpha_f, mu, N_f, C_alpha_f)
            F_tyr, skid_r = LTire(alpha_r, mu, N_r, C_alpha_r)
        
        if tire_model == 'nonlinear': 
            F_tyf, skid_f = NLTire(alpha_f, mu, N_f, C_alpha_f)
            F_tyr, skid_r = NLTire(alpha_r, mu, N_r, C_alpha_r)

        # Map forces from tire to vehicle coordinate system
        F_yf = F_tyf*np.cos(delta_f(t))
        F_xf = F_tyf*np.sin(delta_f(t))
        F_yr = F_tyr*np.cos(delta_r)

        if winch:
            # PID control of winch force
            prev_error_vel = vel_d(t-dt) - vel[-1]
            error_vel = vel_d(t) - y1
            integral_vel.append(error_vel)
            
            prev_error_yaw = yaw_d(t-dt) - yaw[-1]
            error_yaw = yaw_d(t) - y4
            integral_yaw.append(error_yaw)
            
            F_le, F_te = pid(prev_error_vel, error_vel, integral_vel, prev_error_yaw, error_yaw, integral_yaw)
            
            vel.append(y1)
            yaw.append(y4)

            # Winch angles
            gamma_le, gamma_te, X_le, Y_le, X_te, Y_te, le_attach, te_attach = winchAngles(y5, y6, y4, w, l_f)
            
            # Vacuum forces and moment
            F_Vx, F_Vy, M_V = vacuumForce(N_s, F, F_le, F_te, F_yf, F_xf, F_yr, F_g, gamma_le, gamma_te, delta_f(t), y4, l_f, l_r, w)

            # dv_x
            dy1dt = (F_le*np.cos(gamma_le) + F_te*np.cos(gamma_te) - F_xf - F_g*np.cos(y4) + F_Vx)/M + y2*y3
            # dv_y
            dy2dt = (F_le*np.sin(gamma_le) - F_te*np.sin(gamma_te) + F_yf + F_yr + F_g*np.sin(y4) + F_Vy)/M - y1*y3
            # ddgamma
            dy3dt = (l_f*(F_yf + F_le*np.sin(gamma_le) - F_te*np.sin(gamma_te)) - l_r*F_yr + w/2*(F_te*np.cos(gamma_te) - F_le*np.cos(gamma_le)) + M_V)/I_zz
            # dgamma
            dy4dt = y3
            # dX    
            dy5dt = y1*np.cos(y4) - y2*np.sin(y4)
            # dY
            dy6dt = y1*np.sin(y4) + y2*np.cos(y4)
            return [dy1dt, dy2dt, dy3dt, dy4dt, dy5dt, dy6dt]
        
        else:
            # dv_x
            dy1dt = -F_xf/M + y2*y3
            # dv_y
            dy2dt = 1/M*(F_yf + F_yr) - y1*y3
            # ddgamma
            dy3dt = 1/I_zz*(l_f*F_yf - l_r*F_yr)
            # dgamma
            dy4dt = y3
            # dX    
            dy5dt = y1*np.cos(y4) - y2*np.sin(y4)
            # dY
            dy6dt = y1*np.sin(y4) + y2*np.cos(y4)
            return [dy1dt, dy2dt, dy3dt, dy4dt, dy5dt, dy6dt]

    # Termination condition
    def vehicle_stopped(t, Y): return Y[0] - 0.0
    vehicle_stopped.terminal = True
    
    # Solve system
    solution = solve_ivp(dYdt, t_interval, Yzero, events=vehicle_stopped, t_eval=t_span)

    # Solution variables
    #if winch:
    v_x = solution.y[0,:] # y1 = x-direction speed
    v_y = solution.y[1,:] # y2 = y-direction speed
    dgamma = solution.y[2,:] # y3 = yaw rate
    gamma = solution.y[3,:] # y4 = yaw
    X = solution.y[4,:] # y5 = X position
    Y = solution.y[5,:] # y6 = Y position
    
    # Other variables
    t_span = solution.t # time span of solution
    beta = np.arctan(v_y/v_x) #side slip angle
    V = np.sqrt(v_x**2 + v_y**2) # total velocity
    
    # Initialize variables
    delta_f_arr = np.zeros(t_span.size)
    delta_r_arr = np.ones(t_span.size)*delta_r
    alpha_f = np.zeros(t_span.size)
    alpha_r = np.zeros(t_span.size)
    F_tyf = np.zeros(t_span.size)
    F_tyr = np.zeros(t_span.size)
    F_yf = np.zeros(t_span.size)
    F_yr = np.zeros(t_span.size)
    skid_f = np.zeros(t_span.size)
    skid_r = np.zeros(t_span.size)
    
    # Loop to find values
    for i, t in enumerate(t_span):
        # Steer
        delta_f_arr[i] = delta_f(t)
        
        # Slip angles
        if v_x[i] == 0:
            alpha_f[i] = 0
            alpha_r[i] = 0
        else:
            alpha_f[i] = (v_y[i] + l_f*dgamma[i])/v_x[i]  - delta_f(t)
            alpha_r[i] = (v_y[i] - l_r*dgamma[i])/v_x[i]

    # Tire forces
    if tire_model == 'linear':
        for i in range(t_span.size):
            F_tyf[i], skid_f[i] = LTire(alpha_f[i], mu, N_fs, C_alpha_f)
            F_tyr[i], skid_r[i] = LTire(alpha_r[i], mu, N_fs, C_alpha_r)

    if tire_model == 'nonlinear':
        for i in range(t_span.size):
            F_tyf[i], skid_f[i] = NLTire(alpha_f[i], mu, N_fs, C_alpha_f)
            F_tyr[i], skid_r[i] = NLTire(alpha_r[i], mu, N_rs, C_alpha_r)

    F_yf = F_tyf*np.cos(delta_f_arr)
    F_yr = F_tyr*np.cos(delta_r_arr)

    # Needed forces for skid
    F_skid_f = mu*N_fs
    F_skid_r = mu*N_rs

    # Find tire coordinates
    X_f, Y_f, X_r, Y_r = findTire(gamma, l_f, l_r, X, Y)


    gamma_le, gamma_te, X_le, Y_le, X_te, Y_te, le_attach, te_attach = winchAngles(X, Y, gamma, w, l_f)
    
    # Pack values
    F_ty = [F_tyf, F_tyr]
    F_y = [F_yf, F_yr]
    X_t = [X_f, X_r]
    Y_t = [Y_f, Y_r]
    N = [N_fs, N_rs]
    skid = [skid_f, skid_r]
    F_skid = [F_skid_f, F_skid_r]

    return t_span, delta_f_arr, v_x, v_y, V, gamma, dgamma, beta, X, Y, X_t, Y_t, F_ty, F_y, N, skid, F_skid, X_le, Y_le, X_te, Y_te, le_attach, te_attach


def PlotParameters():
    width_large = 8
    height_large = 7
    width = 8
    height = 4
    legend_loc = 'right'
    return width_large, height_large, width, height, legend_loc


# Plotting of a steer
def steerPlot(t_span='NA', delta_f='NA', v_x='NA', v_y='NA', V='NA', gamma='NA', dgamma='NA', dgamma_ss='NA', a_y='NA', beta='NA', X='NA', Y='NA', X_t='NA', Y_t='NA', F_ty='NA', F_y='NA', N='NA', skid='NA', F_skid='NA', X_le='NA', Y_le='NA', X_te='NA', Y_te='NA', le_attach='NA', te_attach='NA', plot_name='noname'):
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    import numpy as np
    
    width_large, height_large, width, height, legend_loc = PlotParameters()
    
    n_plot1 = 0
    plt_number = 0
    if type(v_x).__module__ == np.__name__:
        n_plot1 += 1
        plot1 = v_x
        ylabel = '$v_x$ $[m/s]$'
        title = 'Velocity in the x-direction'
    if type(gamma).__module__ == np.__name__:
        n_plot1 += 1
        plot1 = gamma
        ylabel = '$\gamma$ $[rad/s]$'
        title = 'Yaw'
    if type(delta_f).__module__ == np.__name__:
        n_plot1 += 1
        plot1 = delta_f
        ylabel = '$\delta_f$ $[rad]$'
        title = 'Steering angle'
    if type(X).__module__ == np.__name__:
        n_plot1 += 1
        plot1 = X
        ylabel = '$x$ $[m]$'
        title = 'Position of CG in the x-direction'
    if type(Y).__module__ == np.__name__:
        n_plot1 += 1
        plot1 = X
        ylabel = '$x$ $[m]$'
        title = 'Position of CG in the y-direction'
    if type(v_y).__module__ == np.__name__:
        n_plot1 += 1
        plot1 = v_y
        ylabel = '$v_y$ $[m/s]$'
        title = 'Velocity in the y-direction'
    if type(V).__module__ == np.__name__:
        n_plot1 += 1
        plot1 = V
        ylabel = '$V$ $[m/s]$'
        title = 'Vehicle speed $\\left(V=\\sqrt{u^2 + v^2}\\right)$'
    if type(dgamma).__module__ == np.__name__:
        n_plot1 += 1
        plot1 = dgamma
        ylabel = '$\dot{\gamma}$ $[rad/s]$'
        title = 'Yaw rate'
    if type(beta).__module__ == np.__name__:
        n_plot1 += 1
        plot1 = beta
        ylabel = '$\\beta$ $[rad]$'
        title = 'Side slip angle'
    if type(a_y).__module__ == np.__name__:
        n_plot1 += 1
        plot1 = a_y
        ylabel = '$a_y$ $[g\'s]$'
        title = 'Acceleration in the y-direction'
 
    # Plot
    if n_plot1 == 1:
        fig, ax = plt.subplots(1, figsize=(width, height))

        ax.plot(t_span, plot1, color='tab:blue')
        ax.set_ylabel(ylabel)
        ax.set_xlabel('$t$ $[s]$')
        ax.set_title(title)
        ax.grid(True)
        
    elif n_plot1 > 1:
        fig, ax = plt.subplots(n_plot1, figsize=(width_large, height_large))
        
        if type(v_x).__module__ == np.__name__:
            ax[plt_number].plot(t_span, v_x, color='tab:blue')
            ax[plt_number].grid(True)
            ax[plt_number].set_ylabel('$v_x$ $[m/s]$')
            ax[plt_number].set_title('Vehicle response')
            ax[plt_number].set_xticklabels([])
            plt_number += 1

        if type(gamma).__module__ == np.__name__:
            ax[plt_number].plot(t_span, gamma*180/3.14, color='tab:blue')
            ax[plt_number].grid(True)
            ax[plt_number].set_ylabel('$\\gamma$ $[^{\circ}]$')
            ax[plt_number].set_xticklabels([])
            plt_number += 1

        if type(delta_f).__module__ == np.__name__:
            ax[plt_number].plot(t_span, delta_f*180/3.14, color='tab:blue')
            ax[plt_number].grid(True)
            ax[plt_number].set_ylabel('$\delta_f$ $[^{\circ}]$')
            ax[plt_number].set_xticklabels([])
            plt_number += 1

        if type(X).__module__ == np.__name__:
            ax[plt_number].plot(t_span, X, color='tab:blue')
            ax[plt_number].grid(True)
            ax[plt_number].set_ylabel('$x$ $[m]$')
            ax[plt_number].set_xticklabels([])
            plt_number += 1
        
        if type(Y).__module__ == np.__name__:
            ax[plt_number].plot(t_span, Y, color='tab:blue')
            ax[plt_number].grid(True)
            ax[plt_number].set_ylabel('$y$ $[m]$')
            plt_number += 1

        if type(v_y).__module__ == np.__name__:
            ax[plt_number].plot(t_span, v_y, color='tab:blue')
            ax[plt_number].grid(True)
            ax[plt_number].set_ylabel('$v_y$ $[m/s]$')
            ax[plt_number].set_xticklabels([])
            plt_number += 1
        
        if type(V).__module__ == np.__name__:
            ax[plt_number].plot(t_span, V, color='tab:blue')
            ax[plt_number].grid(True)
            ax[plt_number].set_ylabel('$V$ $[m/s]$')
            ax[plt_number].set_xticklabels([])
            plt_number += 1

        if type(dgamma).__module__ == np.__name__:
            ax[plt_number].plot(t_span, dgamma, color='tab:blue')
            ax[plt_number].grid(True)
            ax[plt_number].set_ylabel('$\\dot{\\gamma}$ $[rad/s]$')
            if type(dgamma_ss).__module__ == np.__name__:
                ax[plt_number].plot(t_span, dgamma_ss, color='tab:orange', linestyle="--", label='$\\dot{\\gamma}_{ss}$')
                ax[plt_number].legend()
            ax[plt_number].set_xticklabels([])
            plt_number += 1

        if type(beta).__module__ == np.__name__:
            ax[plt_number].plot(t_span, beta, color='tab:blue')
            ax[plt_number].grid(True)
            ax[plt_number].set_ylabel('$\\beta$ $[rad]$')
            plt_number += 1

        if type(a_y).__module__ == np.__name__:
            ax[plt_number].plot(t_span, a_y, color='tab:blue')
            ax[plt_number].grid(True)
            ax[plt_number].set_ylabel('$a_y$ $[g\'s]$')
            plt_number += 1
        ax[plt_number-1].set_xlabel('$t$ $[s]$')

        plt.savefig(f'plots/{plot_name}_1.png', bbox_inches='tight')
 
    # Plot normal loads
    if type(N[0]).__module__ == np.__name__:
        fig, ax = plt.subplots(1, figsize=(width, height))

        ax.plot(t_span, N[0], color='tab:blue', label='$N_{fl}$')
        ax.plot(t_span, N[1], color='tab:orange', label='$N_{fr}$', linestyle='--')
        ax.set_ylabel('N $[N]$')
        ax.set_xlabel('$t$ $[s]$')
        ax.legend()
        ax.set_title('Normal loads')
        ax.grid(True)
    
    # Plot tire forces
    if type(F_ty[0]).__module__ == np.__name__:
        fig, ax = plt.subplots(1, figsize=(width, height))

        ax.plot(t_span, F_ty[0], color='tab:blue', label='$F_{tyf}$')
        ax.plot(t_span, F_ty[1], color='tab:orange', label='$F_{tyr}$', linestyle='--')
        ax.set_ylabel('F $[N]$')
        ax.set_xlabel('$t$ $[s]$')
        ax.legend()
        ax.set_title('Tire forces in the y-direction')
        ax.grid(True)
    
    if type(F_y[0]).__module__ == np.__name__:
        fig, ax = plt.subplots(1, figsize=(width, height))

        ax.plot(t_span, F_y[0], color='tab:blue', label='$F_{yf}$')
        ax.plot(t_span, F_y[1], color='tab:orange', label='$F_{yr}$', linestyle='--')
        ax.set_ylabel('F $[N]$')
        ax.set_xlabel('$t$ $[s]$')
        ax.legend()
        ax.set_title('Forces in the y-direction')
        ax.grid(True)
    
    # Plot X and Y
    if type(X[0]).__module__ == np.__name__ and type(Y[0]).__module__ == np.__name__:
        fig, ax = plt.subplots(1, figsize=(width, height))

        ax.plot(X, Y, color='tab:blue')
        ax.set_ylabel('$Y$ $[m]$')
        ax.set_xlabel('$X$ $[m]$')
        ax.axis('equal')
        ax.set_title('Trajectory')
        ax.grid(True)
    
    # Plot X and Y for the tires        
    if type(X_t[0]).__module__ == np.__name__ and type(Y_t[0]).__module__ == np.__name__:
        fig, ax = plt.subplots(1, figsize=(width, height))
    
        ax.plot(X_t[0], Y_t[0], label='Front')
        ax.plot(X_t[1], Y_t[1], label='Rear')
        ax.set_ylabel('$y$ $[m]$')
        ax.set_xlabel('$x$ $[m]$')
        ax.axis('equal')
        ax.set_title('Trajectory of the tires')
        ax.legend()
        ax.grid(True)
        plt.savefig(f'plots/{plot_name}_2.png', bbox_inches='tight')
    
    # Plot X and Y for rope attachment points        
    if type(X_le[0]).__module__ == np.__name__ and type(Y_le[0]).__module__ == np.__name__:
        fig, ax = plt.subplots(1, figsize=(width, height))
        
        ax.plot(X_le, Y_le, label='LE winch')
        ax.plot(X_te, Y_te, label='TE winch')
        ax.scatter(le_attach[0], le_attach[1], label='LE attachment')
        ax.scatter(te_attach[0], te_attach[1], label='TE attachment')
        ax.set_ylabel('$y$ $[m]$')
        ax.set_xlabel('$x$ $[m]$')
        ax.axis('equal')
        ax.set_title('Trajectory of the rope attachments')
        ax.legend()
        ax.grid(True)
        plt.savefig(f'plots/{plot_name}_3.png', bbox_inches='tight')

    # Plot skid
    if type(skid[0]).__module__ == np.__name__:
    
        fig, ax = plt.subplots(1, figsize=(width, height))
    
        ax.plot(t_span, skid[0], label='Front')
        ax.plot(t_span, skid[1], label='Rear')
        ax.set_ylabel('$skid$')
        ax.set_xlabel('$t$ $[s]$')
        ax.set_title('Tire skid')
        ax.legend()
        ax.grid(True)
        plt.savefig(f'plots/{plot_name}_4.png', bbox_inches='tight')
        
    # Plot forces needed for lock up
    if type(F_skid[0]).__module__ == np.__name__:
    
        fig, ax = plt.subplots(1, figsize=(width, height))
    
        ax.plot(t_span, F_skid[0], label='F_skid_f')
        ax.plot(t_span, F_skid[1], label='F_skid_r')
        ax.set_ylabel('$F_skid [N]$')
        ax.set_xlabel('$t$ $[s]$')
        ax.set_title('Maximum forces for no skid $(mu \\cdot N)$')
        ax.legend()
        ax.grid(True)