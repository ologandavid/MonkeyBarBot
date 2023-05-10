function computeA(params, x, ct)
    g = params.g
    l1, l2, L = params.l1, params.l2, params.L 
    I1, I2, I = params.I1, params.I2, params.I
    M, m1, m2 = params.M, params.m1, params.m2
    a1, a2 = params.a1, params.a2
    
    θ1, θ2, px, py, θ = x
    u1, u2 = ct[1:2]
    
    if (u1 == 1)  && (u2 == 1)
        #Both Arms in Contact
        A = [u1*l1*sin(θ+θ1) 0                   1*u1     0       u1*((L/2)*sin(θ)+l1*sin(θ+θ1))
            -u1*l1*cos(θ+θ1) 0                   0        1*u1    u1*((-L/2)*cos(θ)-l1*cos(θ+θ1))
            0                u2*l2*sin(θ+θ2)     1*u2     0       u2*((L/2)*sin(θ)+l2*sin(θ+θ2))
            0               -u2*l2*cos(θ+θ2)     0        1*u2    u2*((-L/2)*cos(θ)-l2*cos(θ+θ2))]
    elseif (u1 == 1) && (u2 != 1)
        # Only Arm 1 in Contact
        A = [u1*l1*sin(θ+θ1) 0                   1*u1     0       u1*((L/2)*sin(θ)+l1*sin(θ+θ1))
            -u1*l1*cos(θ+θ1) 0                   0        1*u1    u1*((-L/2)*cos(θ)-l1*cos(θ+θ1))]
    elseif (u1 != 1) && (u2 == 1)
        # Only Arm 2 in Contact
        A = [0                u2*l2*sin(θ+θ2)     1*u2     0       u2*((L/2)*sin(θ)+l2*sin(θ+θ2))
            0               -u2*l2*cos(θ+θ2)     0        1*u2    u2*((-L/2)*cos(θ)-l2*cos(θ+θ2))]
    end
    return A
end
function monkeybot_dynamics(params, x, τ, ct)
    # Dynamics for monkeybot
    # params = vect
    
    # Unpack params for use in dynamics
    g = params.g
    l1, l2, L = params.l1, params.l2, params.L 
    I1, I2, I = params.I1, params.I2, params.I
    M, m1, m2 = params.M, params.m1, params.m2
    a1, a2 = params.a1, params.a2

    # Unpack the state vector and velocity vector
    # Standard convenction -> x = [q, ̇q], ̇x = [̇q, ̈q]
    θ1, θ2, px, py, θ = x[1:5]
    dθ1, dθ2, dpx, dpy, dθ = x[6:10]
    
    q = [θ1, θ2, px, py, θ]
    dq = [dθ1, dθ2, dpx, dpy, dθ]
    
    # Preallocate Space for xddot
    xdot = zeros(eltype([x;τ]), 10)
    
    # Control
    tau1, tau2 = τ[1:2]
    
    #Arms in Contact represented with u1, u2
    u1, u2 = ct[1:2]
    
    #dx coeff
    c1 = [m1*a1*l1*L*sin(θ1)*dθ 0 0 0 m1*a1*l1*L*sin(θ1)*dθ1
        zeros(4,1) zeros(4,1) zeros(4,1) zeros(4,1) zeros(4,1)]
    c2 = [0 0 0 0 0 
            0 m2*a2*l2*L*sin(θ2)*dθ 0 0 m2*a2*l2*L*sin(θ2)*dθ
            zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)]
    c3 = [-0.5*m1*L*a1*l1*sin(θ1)*dθ^2 0 0 0 -0.5*m1*L*a1*l1*sin(θ1)*(dθ+dθ1)
            zeros(4,1) zeros(4,1) zeros(4,1) zeros(4,1) zeros(4,1)]
    c4 = [0 0 0 0 0 
            0 -0.5*m2*L*a2*l2*sin(θ2)*dθ^2 0 0 -0.5*m2*L*a2*l2*sin(θ2)*(dθ+dθ2)
            zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)]
    Mbar = c1+c2-c3-c4
    
    #ddx coeff
    #3
    c1 = [I1+m1*a1^2*l1^2 0 0 0 m1*a1^2*l1^2+m1*a1*l1*L*cos(θ1)
            zeros(4,1) zeros(4,1) zeros(4,1) zeros(4,1) zeros(4,1)]
    c2 = [0 0 0 0 0 
            0 I2+m2*a2^2*l2^2 0 0 m2*a2^2*l2^2 + m2*a2*l2*L*cos(θ2)
            zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)]
    c3 = zeros(5,5)
    c3[3,3] = M+m1+m2
    c3[4,4] = M+m1+m2
    c3[5,5] = I+I1+I2
    Cbar = c1+ c2 +c3
    
    #2 
    dVdq = [m1*g*a1*l1*cos(θ+θ1)
            m2*g*a2*l2*cos(θ+θ2)
            0
            M*g + m2*g + m1*g
            m1*g*((2/L)*cos(θ) + a1*l1*cos(θ+θ1)) + m2*g*((2/L)*cos(θ) + a2*l2*cos(θ+θ2))]
    
#     #Equation for A^T*λ
#     A_t = [u1*l1*sin(θ+θ1) -u1*l1*cos(θ+θ1) 0 0
#             0 0 u2*l2*sin(θ+θ2) -u2*l2*cos(θ+θ2)
#             1*u1 0 1*u2 0
#             0 1*u1 0 1*u2
#             u1*((L/2)*sin(θ)+l1*sin(θ+θ1)) u1*((-L/2)*cos(θ)-l1*cos(θ+θ1)) u2*((L/2)*sin(θ)+l2*sin(θ+θ2)) u2*((-L/2)*cos(θ)-l2*cos(θ+θ2))]
    A = computeA(params, q, ct)
    na, ma = size(A)
    A_t = transpose(A)


    dvecA_dx = FD.jacobian(_q -> vec(computeA(params, _q, ct)), q)
    dvecA_dt = dvecA_dx*dq
    dA = reshape(dvecA_dt, na, ma)
    @show [Mbar A_t; A zeros(na,na)]
    @show size(vcat([τ; zeros(3,1)]))
    @show size(dVdq)
    @show size(zeros(na,1))
    @show ([vcat([τ; zeros(3,1)]) - dVdq; zeros(na,1)])
    @show -([coeff1; dA]*dq)
    @show size([Mbar A_t; A zeros(na,na)])
    @show size(([τ - dVdq; zeros(na,1)]-([coeff1; dA]*dq)))

    out = [Mbar A_t; A zeros(na,na)]\([τ - dVdq; zeros(na,1)]-([coeff1; dA]*dq))
#     ddq = inv(coeff2)*(u - coeff1*dx-dVdq + A_t*λ) # Only Works if coeff2 isnt singular
    @show 7
    ddq = out[1:5]
    xdot[1:5] = dq
    xdot[6:10] = ddq
    return xdot

end