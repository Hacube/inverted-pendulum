function dxdt = ode_inverted_pendulum(t, x, K_lqr, m_pend, g, L, Is, Kt, R, Ir)
    A = [0, 1, 0; m_pend*g*L/Is, 0, Kt^2/(R*Is); 0, 0, -Kt^2/(R*Ir)];
    B = [0; -Kt/(R*Is); Kt/(R*Ir)];
    dxdt = (A - B * K_lqr) * x;
end
